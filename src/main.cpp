
/*
 * *****************************************************************************
 * *****************************************************************************
 * BXT4_KPL
 * *****************************************************************************
 * Program to control a test rig
 * *****************************************************************************
 * Michael Wettstein
 * Dezember 2022, Zürich
 * *****************************************************************************
 * *****************************************************************************
 */

#include <ArduinoSTL.h> //       https://github.com/mike-matera/ArduinoSTL
#include <Controllino.h> //      PIO Controllino Library
#include <Cylinder.h> //         https://github.com/chischte/cylinder-library
#include <Debounce.h> //         https://github.com/chischte/debounce-library
#include <EEPROM_Counter.h> //   https://github.com/chischte/eeprom-counter-library
#include <Insomnia.h> //         https://github.com/chischte/insomnia-delay-library
#include <Nextion.h> //          PIO Nextion library
#include <SD.h> //               PIO Adafruit SD library

#include <cycle_step.h> //       blueprint of a cycle step
#include <state_controller.h> // keeps track of machine states

// !!!!!!!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!!!--------------|
// !!!!!!!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!!!--------------|
// !!!!!!!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!!!--------------|
//                                                                          |
bool is_in_display_debug_mode = false; // MUST BE FALSE IN PRODUCTION !!!<--|
//                                                                          |
int max_tool_force = 2500; // [N] / 260er->2500 / 450er->4500 --------------|
//                                                                          |
// !!!!!!!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!!!--------------|
// !!!!!!!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!!!--------------|
// !!!!!!!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!! !!!!!!!!!!!!!!!!!!--------------|

// PRE-SETUP SECTION / PIN LAYOUT **********************************************

// INPUT PINS / SENSORS:

const byte DRUCKSENSOR = CONTROLLINO_A7; // 0-10V = 0-12barg
Debounce bandsensor_oben(CONTROLLINO_A0);
Debounce bandsensor_unten(CONTROLLINO_A1);
Debounce taster_startposition(CONTROLLINO_A2);
Debounce taster_endposition(CONTROLLINO_A3);

// OUTPUT PINS / VALVES / MOTORS / RELAYS:
Cylinder zyl_hauptluft(CONTROLLINO_D7);
Cylinder zyl_800_abluft(CONTROLLINO_D1);
Cylinder zyl_800_zuluft(CONTROLLINO_D0);
Cylinder zyl_startklemme(CONTROLLINO_D2);
Cylinder zyl_wippenhebel(CONTROLLINO_D5);
Cylinder zyl_spanntaste(CONTROLLINO_D3);
Cylinder zyl_schweisstaste(CONTROLLINO_D4);
Cylinder zyl_tool_niederhalter(CONTROLLINO_D9);
Cylinder zyl_block_messer(CONTROLLINO_D6);
Cylinder zyl_block_klemmrad(CONTROLLINO_D8);
Cylinder zyl_block_foerdermotor(CONTROLLINO_R5);

Insomnia spinner_step_timeout(500);
Insomnia delay_cycle_step;
Insomnia delay_long_pause;
Insomnia delay_force_update;
Insomnia delay_tacho_update;
Insomnia delay_minimum_filltime;
Insomnia delay_minimum_waittime;

Insomnia timeout_machine_stopped(15000);
Insomnia timeout_reset_button(5000); // pushtime to reset counter

State_controller state_controller;

// GLOBAL VARIABLES ------------------------------------------------------------
// bool (1/0 or true/false)
// byte (0-255)
// int   (-32,768 to 32,767) / unsigned int: 0 to 65,535
// long  (-2,147,483,648 to 2,147,483,647)
// float (6-7 Digits)

byte cycle_step = 0;
byte timeout_count = 0;

unsigned long runtime;
unsigned long runtime_stopwatch;

String error_message = "";

float pressure_float;
int force_int;

// SET UP EEPROM COUNTER ********************************************************
enum eeprom_counter {
  startfuelldruck,
  shorttime_counter,
  longtime_counter,
  cycles_in_a_row,
  long_cooldown_time,
  strap_eject_feed_time,
  end_of_eeprom_enum
};
int number_of_eeprom_values = end_of_eeprom_enum;
int eeprom_min_address = 0;
int eeprom_max_address = 4095;
EEPROM_Counter eeprom_counter;

// DECLARE FUNCTIONS IF NEEDED FOR THE COMPILER: *******************************

String get_main_cycle_display_string();
void reset_flag_of_current_step();
void display_text_in_field(String text, String text_field);
void send_to_nextion();
void clear_text_field(String text_field);
void clear_info_field();
void reset_spinner_picture();

// CREATE VECTOR CONTAINER FOR THE CYCLE STEPS OBJECTS *************************

int Cycle_step::object_count = 0; // enable object counting
std::vector<Cycle_step *> main_cycle_steps;
void reset_flag_of_current_step() { main_cycle_steps[state_controller.get_current_step()]->reset_flags(); }

// NON NEXTION FUNCTIONS *******************************************************

// PNEUMATIC SPTING (800mm CYLINDER) -------------------------------------------

void pneumatic_spring_vent() {
  zyl_800_zuluft.set(0);
  zyl_800_abluft.set(0);
}
void pneumatic_spring_move() {
  zyl_800_zuluft.set(1);
  zyl_800_abluft.set(0); // dont build up pressure
}
void pneumatic_spring_block() {
  zyl_800_zuluft.set(0);
  zyl_800_abluft.set(1);
}

void pneumatic_spring_build_pressure() {
  zyl_800_zuluft.set(1);
  zyl_800_abluft.set(1);
}

// -----------------------------------------------------------------------------

void reset_cylinders() {

  zyl_hauptluft.set(1);
  zyl_wippenhebel.set(0);
  zyl_spanntaste.set(0);
  zyl_schweisstaste.set(0);
  zyl_block_klemmrad.set(0);
  zyl_block_messer.set(0);
  zyl_block_foerdermotor.set(0);
  pneumatic_spring_vent();
}

void reset_state_controller() {
  state_controller.set_machine_stop();
  state_controller.set_step_mode();
  reset_flag_of_current_step();
  state_controller.set_current_step_to(0);
  reset_flag_of_current_step();
  reset_spinner_picture();
}

void reset_machine() {
  reset_cylinders();
  reset_state_controller();
  clear_info_field();
  error_message = "";
  timeout_machine_stopped.reset_time();
}

void stop_machine() {
  zyl_hauptluft.set(0);
  state_controller.set_step_mode();
  state_controller.set_machine_stop();
  reset_cylinders();
}

// NEXTION VARIABLES -----------------------------------------------------------
int nex_current_page;
bool spinner_is_running;
int spinner_pics_array[5] = {22, 23, 24, 25, 26}; // 0-4
int tacho_pics_array[12] = {21, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37}; //0-11

byte nex_current_spinner_pos;
byte current_spinner_pos;
byte nex_current_tacho_pos;
byte current_tacho_pos;

// NEXTION SWITCH STATES LIST --------------------------------------------------
// Every nextion switch button (dualstate) needs a switchstate variable to
// control switchtoggle. Every nextion button (momentary) needs a variable to
// prevent screen flickering.

bool nex_state_zyl_hauptluft;
bool nex_state_zyl_800_zuluft;
bool nex_state_zyl_800_abluft;
bool nex_state_zyl_startklemme;
bool nex_state_zyl_wippenhebel;
bool nex_state_zyl_spanntaste;
bool nex_state_zyl_messer;
bool nex_state_zyl_foerdern;
bool nex_state_zyl_schweisstaste;
bool nex_state_machine_running;
bool nex_state_step_mode = true;
byte nex_state_cycle_step;
int nex_state_force_int;
long nex_state_cycles_in_a_row;
long nex_state_long_cooldown_time;
long nex_state_feed_time;
long nex_state_shorttime_counter;
long nex_state_longtime_counter;
long nex_state_startfuelldruck;
long nex_state_restpausenzeit;
long button_push_stopwatch;
float nex_state_federdruck;
String nex_state_error_message = "INFO";

// NEXTION OBJECTS -------------------------------------------------------------

// PAGE 0:
NexPage nex_page_0 = NexPage(0, 0, "page_0");

// PAGE 1 - LEFT SIDE:
NexPage nex_page_1 = NexPage(1, 0, "page1");
NexButton nex_button_stepback = NexButton(1, 6, "b1");
NexButton nex_button_stepnxt = NexButton(1, 7, "b2");
NexButton nex_button_reset_machine = NexButton(1, 5, "b0");
NexDSButton nex_button_play_pause = NexDSButton(1, 22, "play");
NexDSButton nex_button_mode = NexDSButton(1, 4, "bt1");

// PAGE 1 - RIGHT SIDE
NexDSButton nex_zyl_800_zuluft = NexDSButton(1, 13, "bt5");
NexDSButton nex_zyl_800_abluft = NexDSButton(1, 12, "bt4");
NexDSButton nex_zyl_startklemme = NexDSButton(1, 11, "bt3");
NexButton nex_zyl_wippenhebel = NexButton(1, 10, "b5");
NexButton nex_zyl_spanntaste = NexButton(1, 9, "b4");
NexButton nex_zyl_schweisstaste = NexButton(1, 8, "b3");
NexDSButton nex_zyl_messer = NexDSButton(1, 16, "b6");
NexButton nex_zyl_foerdern = NexButton(1, 17, "b7");
NexButton nex_zyl_hauptluft = NexButton(1, 15, "bt6");

// PAGE 2:
NexPage nex_page_2 = NexPage(2, 0, "page2");
NexButton nex_button_1_left = NexButton(2, 5, "b1");
NexButton nex_button_1_right = NexButton(2, 6, "b2");
NexButton nex_button_2_left = NexButton(2, 8, "b0");
NexButton nex_button_2_right = NexButton(2, 10, "b3");
NexButton nex_button_3_left = NexButton(2, 12, "b4");
NexButton nex_button_3_right = NexButton(2, 14, "b5");
NexButton nex_button_4_left = NexButton(2, 16, "b6");
NexButton nex_button_4_right = NexButton(2, 18, "b7");

// PAGE 3:
NexPage nex_page_3 = NexPage(3, 0, "page3");
NexButton nex_button_reset_shorttime_counter = NexButton(3, 6, "b4");

char buffer[100] = {0}; // This is needed only if you are going to receive a
    // text from the display. You can remove it otherwise.

// NEXTION TOUCH EVENT LISTENERS -----------------------------------------------

NexTouch *nex_listen_list[] = {
    // PAGE 0:
    &nex_page_0,
    // PAGE 1 - LEFT SIDE:
    &nex_page_1, &nex_button_stepback, &nex_button_stepnxt, &nex_button_reset_machine, &nex_button_play_pause,
    &nex_button_mode,
    // PAGE 1 - RIGHT SIDE:
    &nex_zyl_foerdern, &nex_zyl_messer, &nex_zyl_startklemme, &nex_zyl_800_zuluft, &nex_zyl_800_abluft,
    &nex_zyl_wippenhebel, &nex_zyl_spanntaste, &nex_zyl_schweisstaste, &nex_zyl_hauptluft,
    // PAGE 2:
    &nex_page_2, &nex_button_1_left, &nex_button_1_right, &nex_button_2_left, &nex_button_2_right, &nex_button_3_left,
    &nex_button_3_right, &nex_button_4_left, &nex_button_4_right,
    // PAGE 3:
    &nex_page_3, &nex_button_reset_shorttime_counter, //
    NULL};

// NEXTION TOUCH EVENT FUNCTIONS -----------------------------------------------

// TOUCH EVENT FUNCTIONS PAGE CHANGES ------------------------------------------

void nex_page_0_push_callback(void *ptr) { nex_current_page = 0; }

void nex_page_1_push_callback(void *ptr) {
  nex_current_page = 1;

  // REFRESH BUTTON STATES:
  nex_state_cycle_step = -1;
  nex_state_step_mode = 1;
  nex_state_error_message = "INFO";
  nex_state_zyl_800_zuluft = 0;
  nex_state_zyl_800_abluft = 1; // INVERTED VALVE LOGIC
  nex_state_zyl_startklemme = 0;
  nex_state_zyl_wippenhebel = 0;
  nex_state_zyl_spanntaste = 0;
  nex_state_zyl_messer = 0;
  nex_state_zyl_foerdern = 0;
  nex_state_zyl_schweisstaste = 0;
  nex_state_machine_running = 0;
  nex_state_zyl_hauptluft = 0;
  nex_state_force_int = -5;
  nex_current_spinner_pos = 1;
  nex_current_tacho_pos = 0;
}

void nex_page_2_push_callback(void *ptr) {
  nex_current_page = 2;
  // REFRESH BUTTON STATES:
  nex_state_cycles_in_a_row = 0;
  nex_state_long_cooldown_time = 0;
  nex_state_feed_time = 0;
  nex_state_startfuelldruck = 0;
  nex_state_federdruck = 10000;
}

void nex_page_3_push_callback(void *ptr) {
  nex_current_page = 3;
  // REFRESH BUTTON STATES:
  nex_state_shorttime_counter = 0;
  nex_state_longtime_counter = 0;
}

// TOUCH EVENT FUNCTIONS PAGE 1 - LEFT SIDE ------------------------------------

void nex_button_play_pause_push_callback(void *ptr) { //
  state_controller.toggle_machine_running_state();
  nex_state_machine_running = !nex_state_machine_running;
}

void nex_button_play_pause_pop_callback(void *ptr) {}

void nex_button_mode_push_callback(void *ptr) {
  state_controller.toggle_step_auto_mode();
  nex_state_step_mode = state_controller.is_in_step_mode();
}
void nex_button_stepback_push_callback(void *ptr) {
  state_controller.set_machine_stop();
  reset_flag_of_current_step();
  state_controller.set_step_mode();
  state_controller.switch_to_previous_step();
  reset_flag_of_current_step();
}
void nex_button_stepnxt_push_callback(void *ptr) {
  state_controller.set_machine_stop();
  reset_flag_of_current_step();
  state_controller.set_step_mode();
  state_controller.switch_to_next_step();
  reset_flag_of_current_step();
}
void nex_button_reset_machine_push_callback(void *ptr) { reset_machine(); }

// TOUCH EVENT FUNCTIONS PAGE 1 - RIGHT SIDE -----------------------------------

void nex_zyl_800_zuluft_push_callback(void *ptr) {
  zyl_800_zuluft.set(1);
  nex_state_zyl_800_zuluft = !nex_state_zyl_800_zuluft;
}
void nex_zyl_800_zuluft_pop_callback(void *ptr) { //
  zyl_800_zuluft.set(0);
}

void nex_zyl_800_abluft_push_callback(void *ptr) {
  zyl_800_abluft.toggle();
  nex_state_zyl_800_abluft = !nex_state_zyl_800_abluft;
}

void nex_zyl_startklemme_push_callback(void *ptr) {
  zyl_startklemme.toggle();
  nex_state_zyl_startklemme = !nex_state_zyl_startklemme;
}

void nex_zyl_wippenhebel_push_callback(void *ptr) {
  zyl_wippenhebel.toggle();
  nex_state_zyl_wippenhebel = 1;
}

void nex_zyl_wippenhebel_pop_callback(void *ptr) { //
  nex_state_zyl_wippenhebel = 0;
}

void nex_zyl_spanntaste_push_callback(void *ptr) { zyl_spanntaste.set(1); }

void nex_zyl_spanntaste_pop_callback(void *ptr) { zyl_spanntaste.set(0); }

void nex_zyl_schweisstaste_push_callback(void *ptr) { zyl_schweisstaste.set(1); }

void nex_zyl_schweisstaste_pop_callback(void *ptr) { zyl_schweisstaste.set(0); }

void nex_zyl_messer_push_callback(void *ptr) { zyl_block_messer.set(1); }

void nex_zyl_messer_pop_callback(void *ptr) { zyl_block_messer.set(0); }

void nex_zyl_foerdern_push_callback(void *ptr) {
  zyl_block_klemmrad.set(1);
  zyl_block_foerdermotor.set(1);
}

void nex_zyl_foerdern_pop_callback(void *ptr) {
  zyl_block_klemmrad.set(0);
  zyl_block_foerdermotor.set(0);
}

void nex_zyl_hauptluft_push_callback(void *ptr) {
  zyl_hauptluft.toggle();
  nex_state_zyl_hauptluft = !nex_state_zyl_hauptluft;
}

// TOUCH EVENT FUNCTIONS PAGE 2 ------------------------------------------------

void decrease_slider_value(int eeprom_value_number, long min_value, long interval) {
  long current_value = eeprom_counter.get_value(eeprom_value_number);

  if (current_value >= (min_value + interval)) {
    eeprom_counter.set_value(eeprom_value_number, (current_value - interval));
  } else {
    eeprom_counter.set_value(eeprom_value_number, min_value);
  }
}

void increase_slider_value(int eeprom_value_number, long max_value, long interval) {
  long current_value = eeprom_counter.get_value(eeprom_value_number);

  if (current_value <= (max_value - interval)) {
    eeprom_counter.set_value(eeprom_value_number, (current_value + interval));
  } else {
    eeprom_counter.set_value(eeprom_value_number, max_value);
  }
}

void nex_button_1_left_push_callback(void *ptr) { decrease_slider_value(cycles_in_a_row, 0, 1); }

void nex_button_1_right_push_callback(void *ptr) { increase_slider_value(cycles_in_a_row, 10, 1); }

void nex_button_2_left_push_callback(void *ptr) { decrease_slider_value(long_cooldown_time, 0, 10); }

void nex_button_2_right_push_callback(void *ptr) { increase_slider_value(long_cooldown_time, 600, 10); }

void nex_button_3_left_push_callback(void *ptr) { decrease_slider_value(strap_eject_feed_time, 0, 100); }

void nex_button_3_right_push_callback(void *ptr) { increase_slider_value(strap_eject_feed_time, 2000, 100); }

void nex_button_4_left_push_callback(void *ptr) { decrease_slider_value(startfuelldruck, 0, 100); }

void nex_button_4_right_push_callback(void *ptr) { increase_slider_value(startfuelldruck, 3000, 100); }

// TOUCH EVENT FUNCTIONS PAGE 3 ------------------------------------------------

void nex_button_reset_shorttime_counter_push_callback(void *ptr) {
  eeprom_counter.set_value(shorttime_counter, 0);

  // RESET LONGTIME COUNTER IF RESET BUTTON IS PRESSED LONG ENOUGH:
  // ACTIVATE TIMEOUT TO RESET LONGTIME COUNTER:
  timeout_reset_button.reset_time();
  timeout_reset_button.set_flag_activated(1);
}

void nex_button_reset_shorttime_counter_pop_callback(void *ptr) { timeout_reset_button.set_flag_activated(0); }

// END OF NEXTION TOUCH EVENT FUNCTIONS ****************************************

// NEXTION SETUP ***************************************************************

void nextion_setup() {
  Serial2.begin(9600);

  // REGISTER EVENT CALLBACK FUNCTIONS -----------------------------------------

  // PAGE 0:
  nex_page_0.attachPush(nex_page_0_push_callback);
  // PAGE 1 - LEFT SIDE:
  nex_page_1.attachPush(nex_page_1_push_callback);
  nex_button_stepback.attachPush(nex_button_stepback_push_callback);
  nex_button_stepnxt.attachPush(nex_button_stepnxt_push_callback);
  nex_button_reset_machine.attachPush(nex_button_reset_machine_push_callback);
  nex_button_stepback.attachPush(nex_button_stepback_push_callback);
  nex_button_stepnxt.attachPush(nex_button_stepnxt_push_callback);
  nex_button_mode.attachPush(nex_button_mode_push_callback);
  nex_button_play_pause.attachPush(nex_button_play_pause_push_callback);
  nex_button_play_pause.attachPop(nex_button_play_pause_pop_callback);
  // PAGE 1 - RIGHT SIDE:
  nex_zyl_startklemme.attachPush(nex_zyl_startklemme_push_callback);
  nex_zyl_800_zuluft.attachPush(nex_zyl_800_zuluft_push_callback);
  nex_zyl_800_zuluft.attachPop(nex_zyl_800_zuluft_pop_callback);
  nex_zyl_800_abluft.attachPush(nex_zyl_800_abluft_push_callback);
  nex_zyl_startklemme.attachPush(nex_zyl_startklemme_push_callback);
  nex_zyl_wippenhebel.attachPush(nex_zyl_wippenhebel_push_callback);
  nex_zyl_wippenhebel.attachPop(nex_zyl_wippenhebel_pop_callback);
  nex_zyl_spanntaste.attachPush(nex_zyl_spanntaste_push_callback);
  nex_zyl_spanntaste.attachPop(nex_zyl_spanntaste_pop_callback);
  nex_zyl_schweisstaste.attachPush(nex_zyl_schweisstaste_push_callback);
  nex_zyl_schweisstaste.attachPop(nex_zyl_schweisstaste_pop_callback);
  nex_zyl_messer.attachPush(nex_zyl_messer_push_callback);
  nex_zyl_messer.attachPop(nex_zyl_messer_pop_callback);
  nex_zyl_foerdern.attachPush(nex_zyl_foerdern_push_callback);
  nex_zyl_foerdern.attachPop(nex_zyl_foerdern_pop_callback);
  nex_zyl_hauptluft.attachPush(nex_zyl_hauptluft_push_callback);
  // PAGE 2:
  nex_page_2.attachPush(nex_page_2_push_callback);
  nex_button_1_left.attachPush(nex_button_1_left_push_callback);
  nex_button_1_right.attachPush(nex_button_1_right_push_callback);
  nex_button_2_left.attachPush(nex_button_2_left_push_callback);
  nex_button_2_right.attachPush(nex_button_2_right_push_callback);
  nex_button_3_left.attachPush(nex_button_3_left_push_callback);
  nex_button_3_right.attachPush(nex_button_3_right_push_callback);
  nex_button_4_left.attachPush(nex_button_4_left_push_callback);
  nex_button_4_right.attachPush(nex_button_4_right_push_callback);

  // PAGE 3:
  nex_page_3.attachPush(nex_page_3_push_callback);
  nex_button_reset_shorttime_counter.attachPush(nex_button_reset_shorttime_counter_push_callback);
  nex_button_reset_shorttime_counter.attachPop(nex_button_reset_shorttime_counter_pop_callback);

  // ---------------------------------------------------------------------------

  delay(3000); // Show start screen
  sendCommand("page 1");
  send_to_nextion();

} // END OF NEXTION SETUP

// NEXTION GENERAL DISPLAY FUNCTIONS *******************************************

void send_to_nextion() {
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
}

void show_info_field() {
  if (nex_current_page == 1) {
    Serial2.print("vis t4,1");
    send_to_nextion();
  }
}

String add_suffix_to_value(long value, String suffix) {
  String value_string = String(value);
  String space = " ";
  String suffixed_string = value_string + space + suffix;
  return suffixed_string;
}

void display_text_in_info_field(String text) {
  if (nex_current_page == 1) {
    Serial2.print("t4");
    Serial2.print(".txt=");
    Serial2.print("\"");
    Serial2.print(text);
    Serial2.print("\"");
    send_to_nextion();
  }
}

void hide_info_field() {
  if (nex_current_page == 1) {
    Serial2.print("vis t4,0");
    send_to_nextion();
  }
}

void clear_text_field(String text_field) {
  Serial2.print(text_field);
  Serial2.print(".txt=");
  Serial2.print("\"");
  Serial2.print(""); // erase text
  Serial2.print("\"");
  send_to_nextion();
}

void clear_info_field() {
  if (nex_current_page == 1) {
    clear_text_field("t4");
  }
}

void display_value_in_field(int value, String value_field) {
  Serial2.print(value_field);
  Serial2.print(".val=");
  Serial2.print(value);
  send_to_nextion();
}

void display_text_in_field(String text, String text_field) {
  Serial2.print(text_field);
  Serial2.print(".txt=");
  Serial2.print("\"");
  Serial2.print(text);
  Serial2.print("\"");
  send_to_nextion();
}

void toggle_ds_switch(String button) {
  Serial2.print("click " + button + ",1");
  send_to_nextion();
}

void set_momentary_button_high_or_low(String button, bool state) {
  Serial2.print("click " + button + "," + state);
  send_to_nextion();
}

// NEXTION DISPLAY LOOPS *******************************************************

// DISPLAY LOOP PAGE 1 LEFT SIDE: ----------------------------------------------

void display_pic_in_field(String picture, String textField) {
  Serial2.print(textField);
  Serial2.print(".pic=");
  Serial2.print(picture);
  send_to_nextion();
  // 2nd picture for buttons:
  Serial2.print(textField);
  Serial2.print(".pic2=");
  Serial2.print(picture);
  send_to_nextion();
}

void reset_spinner_picture() { //
  current_spinner_pos = 0;
}

void update_spinner_picture() {
  if (nex_current_spinner_pos != current_spinner_pos) {
    String picture = String(spinner_pics_array[current_spinner_pos]);
    display_pic_in_field(picture, "spinner");
    nex_current_spinner_pos = current_spinner_pos;
  }
}

void select_next_spinner_pic() {
  int number_of_spinner_pics = sizeof(spinner_pics_array) / sizeof(int);
  int max_spinner_pic_pos = number_of_spinner_pics - 1;

  current_spinner_pos++;
  if (current_spinner_pos > max_spinner_pic_pos) {
    current_spinner_pos = 1;
  }
}

void run_spinner() {
  if (spinner_is_running) {
    if (spinner_step_timeout.has_timed_out()) {
      select_next_spinner_pic();
      spinner_step_timeout.reset_time();
    }
  }
}

void update_tacho_picture() {
  if (nex_current_tacho_pos != current_tacho_pos) {
    String picture = String(tacho_pics_array[current_tacho_pos]);
    display_pic_in_field(picture, "force");
    nex_current_tacho_pos = current_tacho_pos;
  }
}

int get_tacho_pos_from_pressure() {

  int number_of_tacho_pics = sizeof(tacho_pics_array) / sizeof(int);
  int max_tacho_pic_number = number_of_tacho_pics - 1;

  float force_fraction = float(force_int) / max_tool_force;
  int array_position = int(round(force_fraction * float(max_tacho_pic_number)));

  if (array_position >= max_tacho_pic_number) {
    array_position = max_tacho_pic_number;
  }

  return array_position;
}

void run_tacho() { //
  current_tacho_pos = get_tacho_pos_from_pressure();
};

String get_main_cycle_display_string() {
  int current_step = state_controller.get_current_step();
  String display_text_cycle_name = main_cycle_steps[current_step]->get_display_text();
  return display_text_cycle_name;
}

void update_cycle_name() {
  if (nex_state_cycle_step != state_controller.get_current_step()) {
    String number = String(state_controller.get_current_step() + 1);
    String name = get_main_cycle_display_string();
    Serial.println(number + " " + name);
    display_text_in_field(number + " " + name, "t0");
    nex_state_cycle_step = state_controller.get_current_step();
  }
}

void update_button_play_pause() {
  if (nex_state_machine_running != state_controller.machine_is_running()) {
    toggle_ds_switch("play");
    nex_state_machine_running = state_controller.machine_is_running();
  }
}

void update_button_step_auto() {
  if (nex_state_step_mode != state_controller.is_in_step_mode()) {
    toggle_ds_switch("bt1");
    nex_state_step_mode = state_controller.is_in_step_mode();
  }
}

void update_force_display() {
  if (nex_state_force_int != force_int) {
    String suffixed_value = add_suffix_to_value(force_int, "N");
    display_text_in_field(suffixed_value, "t2");
    nex_state_force_int = force_int;
  }
}

void show_error() {
  if (nex_state_error_message != error_message) {
    display_text_in_info_field(error_message);
  }
  nex_state_error_message = error_message;
}

void show_remaining_pause_time() {
  long restpausenzeit = delay_long_pause.get_remaining_delay_time() / 1000;

  if (nex_state_restpausenzeit != restpausenzeit) {
    if (restpausenzeit > 0 && restpausenzeit < 1000) {
      String pause = "PAUSE: ";
      String zeit = add_suffix_to_value(restpausenzeit, "s");
      String displaystring = pause + zeit;
      display_text_in_info_field(displaystring);
      nex_state_restpausenzeit = restpausenzeit;
    } else {
      display_text_in_info_field("");
    }
  }
}

void display_loop_page_1_left_side() {

  run_spinner();
  update_spinner_picture();

  if (delay_tacho_update.delay_time_is_up(200)) {
    run_tacho();
    update_tacho_picture();
    update_force_display();
  }

  update_cycle_name();

  update_button_play_pause();

  update_button_step_auto();

  show_error();

  show_remaining_pause_time();
}

// DISPLAY LOOP PAGE 1 RIGHT SIDE: ---------------------------------------------

void update_button_zuluft_800() {
  if (zyl_800_zuluft.get_state() != nex_state_zyl_800_zuluft) {
    toggle_ds_switch("bt5");
    nex_state_zyl_800_zuluft = !nex_state_zyl_800_zuluft;
  }
}

void update_button_abluft_800() {
  if (zyl_800_abluft.get_state() != nex_state_zyl_800_abluft) {
    toggle_ds_switch("bt4");
    nex_state_zyl_800_abluft = !nex_state_zyl_800_abluft;
  }
}

void update_button_klemmblock() {
  if (zyl_startklemme.get_state() != nex_state_zyl_startklemme) {
    toggle_ds_switch("bt3");
    nex_state_zyl_startklemme = !nex_state_zyl_startklemme;
  }
}

void update_button_wippenhebel() {
  if (zyl_wippenhebel.get_state() != nex_state_zyl_wippenhebel) {
    bool state = zyl_wippenhebel.get_state();
    set_momentary_button_high_or_low("b5", state);
    nex_state_zyl_wippenhebel = state;
  }
}

void update_button_spanntaste() {
  if (zyl_spanntaste.get_state() != nex_state_zyl_spanntaste) {
    bool state = zyl_spanntaste.get_state();
    set_momentary_button_high_or_low("b4", state);
    nex_state_zyl_spanntaste = state;
  }
}

void update_button_messer() {
  if (zyl_block_messer.get_state() != nex_state_zyl_messer) {
    bool state = zyl_block_messer.get_state();
    set_momentary_button_high_or_low("b6", state);
    nex_state_zyl_messer = state;
  }
}

void update_button_foerdern() {
  bool state = zyl_block_foerdermotor.get_state();
  if (state != nex_state_zyl_foerdern) {
    set_momentary_button_high_or_low("b7", state);
    nex_state_zyl_foerdern = state;
  }
}

void update_button_schweissen() {
  if (zyl_schweisstaste.get_state() != nex_state_zyl_schweisstaste) {
    bool state = zyl_schweisstaste.get_state();
    set_momentary_button_high_or_low("b3", state);
    nex_state_zyl_schweisstaste = state;
  }
}

void update_button_hauptluft() {
  if (zyl_hauptluft.get_state() != nex_state_zyl_hauptluft) {
    toggle_ds_switch("bt6");
    nex_state_zyl_hauptluft = zyl_hauptluft.get_state();
  }
}

void display_loop_page_1_right_side() {
  update_button_zuluft_800();
  update_button_abluft_800();
  update_button_klemmblock();
  update_button_wippenhebel();
  update_button_spanntaste();
  update_button_messer();
  update_button_foerdern();
  update_button_schweissen();
  update_button_hauptluft();
}

// DIPLAY LOOP PAGE 2: ---------------------------------------------------------

void update_number_of_cycles() {
  long value = eeprom_counter.get_value(cycles_in_a_row);
  if (nex_state_cycles_in_a_row != value) {
    String text = String(value);
    display_text_in_field(text, "t4");
    nex_state_cycles_in_a_row = value;
  }
}

void update_cooldown_time() {
  long value = eeprom_counter.get_value(long_cooldown_time);
  if (nex_state_long_cooldown_time != value) {
    String text = add_suffix_to_value(value, "s");
    display_text_in_field(text, "t5");
    nex_state_long_cooldown_time = value;
  }
}

void update_strap_feed_time() {
  long value = eeprom_counter.get_value(strap_eject_feed_time);
  if (nex_state_feed_time != value) {
    String text = add_suffix_to_value(value, "ms");
    display_text_in_field(text, "t7");
    nex_state_feed_time = value;
  }
}

void update_startfuelldruck() {
  if (nex_state_startfuelldruck != eeprom_counter.get_value(startfuelldruck)) {
    int value = eeprom_counter.get_value(startfuelldruck);
    String display_string = add_suffix_to_value(value, "N");
    display_text_in_field(display_string, "t9");
    nex_state_startfuelldruck = eeprom_counter.get_value(startfuelldruck);
  }
}
void update_pressure_display() {
  Serial2.print("t10.txt=");
  Serial2.print("\"");
  Serial2.print(pressure_float, 1);
  Serial2.print(" bar");
  Serial2.print("\"");
  send_to_nextion();
  nex_state_federdruck = pressure_float;
}

void display_loop_page_2() {
  update_number_of_cycles();
  update_cooldown_time();
  update_strap_feed_time();
  update_startfuelldruck();

  if (delay_force_update.delay_time_is_up(200)) {
    update_pressure_display();
  }
}

// DIPLAY LOOP PAGE 3: ---------------------------------------------------------

void update_longtime_counter_value() {
  long value = eeprom_counter.get_value(shorttime_counter);
  if (nex_state_shorttime_counter != value) {
    display_text_in_field(String(value), "t12");
    nex_state_shorttime_counter = value;
  }
}

void reset_longtime_counter_value() {
  if (timeout_reset_button.is_marked_activated()) {
    if (timeout_reset_button.has_timed_out()) {
      eeprom_counter.set_value(longtime_counter, 0);
    }
  }
}

void update_shorttime_counter_value() {
  long value = eeprom_counter.get_value(longtime_counter);
  if (nex_state_longtime_counter != value) {
    display_text_in_field(String(value), "t10");
    nex_state_longtime_counter = value;
  }
}

void display_loop_page_3() {

  update_longtime_counter_value();
  reset_longtime_counter_value();
  update_shorttime_counter_value();
}

// NEXTION MAIN LOOP: ----------------------------------------------------------

void nextion_loop() {

  nexLoop(nex_listen_list); // check for any touch event

  // PAGE 1 --------------------------------------
  if (nex_current_page == 1) // START PAGE 1
  {
    display_loop_page_1_left_side();
    display_loop_page_1_right_side();
  }
  // PAGE 2 --------------------------------------
  if (nex_current_page == 2) // START PAGE 2
  {
    display_loop_page_2();
  }

  // PAGE 3 --------------------------------------
  if (nex_current_page == 3) { // START PAGE 3
    display_loop_page_3();
  }

} // END OF NEXTION MAIN LOOP

// PROCESS PRESSURE SENSOR -----------------------------------------------------

float get_pressure_from_sensor() {
  // DRUCKSENSOR 0-10V => 0-12bar
  // CONTROLLINO ANALOG INPUT VALUE 0-1023, 30mV per digit (controlino.biz)
  // 10V   => analogRead 333.3 (10V/30mV)
  // 12bar => anlaogRead 333.3
  // 1bar  => analogRead 27.778
  return analogRead(DRUCKSENSOR) / 27.778; //[bar]
}

float smoothe_measurement(float pressure_float) {
  static float pressure_float_smoothed;
  pressure_float_smoothed = ((pressure_float_smoothed * 4 + pressure_float) / 5);
  return pressure_float_smoothed;
}

float calm_measurement(float pressure_float) {
  // To prevent flickering, the pressure value will only be updated
  // if there's a higher or lower value five times in a row.
  // A positive calmcounter indicates rising pressure.
  // A negative calmcounter indicates dropping pressure.

  static float calmcounter;
  static float pressure_sum;
  static float pressure_calmed;
  static float prev_pressure_calmed;

  // Pressure seems to rise:
  if (pressure_float > pressure_calmed) {
    if (calmcounter >= 0) {
      calmcounter++;
      pressure_sum += pressure_float;
    }
    if (calmcounter < 0) // Pressure seemed to drop last time, reset calmcounter
    {
      calmcounter = 0;
      pressure_sum = 0;
    }
  }
  // Pressure seems to fall:
  if (pressure_float < pressure_calmed) {
    if (calmcounter <= 0) {
      calmcounter--;
      pressure_sum += pressure_float;
    }
    if (calmcounter > 0) // Pressure seemed to rise last time, reset calmcounter
    {
      calmcounter = 0;
      pressure_sum = 0;
    }
  }

  if (fabs(calmcounter) >= 5) //
  {
    static float min_difference = 0.00; // [bar]
    // Update value only if there is a significant difference to the previous
    // value:
    if (fabs(pressure_sum / fabs(calmcounter) - prev_pressure_calmed) > min_difference) {
      pressure_calmed = pressure_sum / fabs(calmcounter);
      prev_pressure_calmed = pressure_calmed;
    }
    calmcounter = 0;
    pressure_sum = 0;
  }
  return pressure_calmed;
}

int convert_pressure_to_force(float pressure) {

  // Calculate force:
  int force = pressure * 1472.6; // 1bar  => 1472.6N (Dauertest BXT 3-32 Zylinderkraft.xlsx)

  // Set last digit zero:
  force = force / 10;
  force = force * 10;

  return force;
}

void read_and_process_pressure() {
  pressure_float = get_pressure_from_sensor(); //[bar]
  pressure_float = smoothe_measurement(pressure_float); //[bar]
  pressure_float = calm_measurement(pressure_float);
  force_int = convert_pressure_to_force(pressure_float); // [N]
}

// CREATE CYCLE STEP CLASSES ***************************************************
// -----------------------------------------------------------------------------
class Aufwecken : public Cycle_step {
  String get_display_text() { return "AUFWECKEN"; }

  void do_initial_stuff(){};
  void do_loop_stuff() {
    zyl_wippenhebel.stroke(1500, 1000);

    if (zyl_wippenhebel.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Vorschieben : public Cycle_step {
  String get_display_text() { return "VORSCHIEBEN"; }
  long feed_time;

  void do_initial_stuff() {
    feed_time = eeprom_counter.get_value(strap_eject_feed_time);
    delay_cycle_step.set_unstarted();
    zyl_wippenhebel.set(1);
    zyl_block_klemmrad.set(1);
    zyl_block_foerdermotor.set(1);
  };
  void do_loop_stuff() {
    if (delay_cycle_step.delay_time_is_up(feed_time)) {
      zyl_wippenhebel.set(0);
      // zyl_block_klemmrad.set(0);
      zyl_block_foerdermotor.set(0);
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Schneiden : public Cycle_step {
  String get_display_text() { return "SCHNEIDEN"; }

  void do_initial_stuff(){};
  void do_loop_stuff() {
    zyl_block_messer.stroke(1500, 500);

    if (zyl_block_messer.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Stirzel : public Cycle_step {
  String get_display_text() { return "STIRZEL"; }

  void do_initial_stuff() {
    zyl_block_klemmrad.set(1);
    zyl_block_foerdermotor.set(1);
    delay_cycle_step.set_unstarted();
  };
  void do_loop_stuff() {
    if (delay_cycle_step.delay_time_is_up(500)) {
      zyl_block_klemmrad.set(0);
      zyl_block_foerdermotor.set(0);
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Festklemmen : public Cycle_step {
  String get_display_text() { return "FESTKLEMMEN"; }

  void do_initial_stuff() {
    zyl_startklemme.set(1);
    delay_cycle_step.set_unstarted();
  };
  void do_loop_stuff() {
    if (delay_cycle_step.delay_time_is_up(400)) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Startdruck : public Cycle_step {
  String get_display_text() { return "STARTDRUCK"; }
  byte is_full_counter = 0;
  int minimum_inflation = 20; // [N]

  void do_initial_stuff() {
    delay_cycle_step.set_unstarted();
    is_full_counter = 0;
  };

  void do_loop_stuff() {

    // Build pressure after minmum wait time
    if (force_int + minimum_inflation <= eeprom_counter.get_value(startfuelldruck)) {
      if (delay_minimum_waittime.delay_time_is_up(500)) {
        pneumatic_spring_build_pressure();
        delay_minimum_filltime.reset_time();
        is_full_counter = 0;
      }
    }
    // Stop building pressure after minmum filltime
    else {
      if (delay_minimum_filltime.delay_time_is_up(100)) {
        pneumatic_spring_block();
        delay_minimum_waittime.reset_time();
        is_full_counter++;
      }
    }

    if (is_full_counter >= 20) {
      pneumatic_spring_block();
      set_loop_completed();
    };
  };
};
// -----------------------------------------------------------------------------
class Spannen : public Cycle_step {
  String get_display_text() { return "SPANNEN"; }

  void do_initial_stuff() {
    pneumatic_spring_block();
    zyl_spanntaste.set(1); // Spanntaste betätigen
    zyl_block_klemmrad.set(0);
    delay_cycle_step.set_unstarted();
  };
  void do_loop_stuff() {
    if (is_in_display_debug_mode) {
      set_loop_completed();
    };
    if (taster_endposition.get_raw_button_state()) {
      if (delay_cycle_step.delay_time_is_up(400)) {
        set_loop_completed();
      }
    };
  };
};
// -----------------------------------------------------------------------------
class Schweissen : public Cycle_step {
  String get_display_text() { return "SCHWEISSEN"; }

  void do_initial_stuff() { zyl_spanntaste.set(0); };

  void do_loop_stuff() {
    zyl_schweisstaste.stroke(1000, 2000);

    if (zyl_schweisstaste.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
// Abkühlen und Druck abbauen
class Abkuehlen : public Cycle_step {
  String get_display_text() { return "ENTLUEFTEN"; }

  void do_initial_stuff() {
    delay_cycle_step.set_unstarted();
    zyl_block_klemmrad.set(0);
    pneumatic_spring_vent();
  };
  void do_loop_stuff() {
    if (pressure_float < 0.1) // warten bis der Druck abgebaut ist
    {
      if (delay_cycle_step.delay_time_is_up(4000)) { // Restluft kann entweichen
        set_loop_completed();
      }
    }
  };
};
// -----------------------------------------------------------------------------
class Wippenhebel : public Cycle_step {
  String get_display_text() { return "WIPPENHEBEL"; }

  void do_initial_stuff(){};
  void do_loop_stuff() {
    zyl_wippenhebel.stroke(1500, 1000);

    if (zyl_wippenhebel.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Entspannen : public Cycle_step {
  String get_display_text() { return "ENTSPANNEN"; }

  void do_initial_stuff() {
    delay_cycle_step.set_unstarted();
    zyl_startklemme.set(0);
  };
  void do_loop_stuff() {
    if (delay_cycle_step.delay_time_is_up(500)) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------

class Zurueckfahren : public Cycle_step {
  String get_display_text() { return "ZURUECKFAHREN"; }

  void do_initial_stuff() {
    pneumatic_spring_move();
    delay_cycle_step.set_unstarted();
  };
  void do_loop_stuff() {
    if (is_in_display_debug_mode) {
      eeprom_counter.count_one_up(shorttime_counter);
      eeprom_counter.count_one_up(longtime_counter);
      set_loop_completed();
    };
    if (taster_startposition.get_raw_button_state()) {
      pneumatic_spring_vent();
      if (pressure_float < 0.1) // warten bis der Druck abgebaut ist
      {
        if (delay_cycle_step.delay_time_is_up(500)) {
          eeprom_counter.count_one_up(shorttime_counter);
          eeprom_counter.count_one_up(longtime_counter);
          set_loop_completed();
        }
      }
    }
  };
};
// -----------------------------------------------------------------------------
class Pause : public Cycle_step {
  String get_display_text() { return "PAUSE"; }
  byte testZyklenZaehler;
  long abkuehldauer;

  void do_initial_stuff() {
    timeout_count = 0;
    error_message = "";
    testZyklenZaehler++;
    delay_long_pause.set_unstarted();
    abkuehldauer = eeprom_counter.get_value(long_cooldown_time) * 1000;
  };

  void do_loop_stuff() {
    if (testZyklenZaehler >= eeprom_counter.get_value(cycles_in_a_row)) {
      timeout_machine_stopped.reset_time(); // Deactivate timeout error during pause.
      if (delay_long_pause.delay_time_is_up(abkuehldauer)) {
        testZyklenZaehler = 0;
        set_loop_completed();
      }
    } else {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// SETUP LOOP ------------------------------------------------------------------

void setup() {
  eeprom_counter.setup(eeprom_min_address, eeprom_max_address, number_of_eeprom_values);

  Serial.begin(115200);

  nextion_setup();

  pinMode(DRUCKSENSOR, INPUT);

  delay(2000);

  //------------------------------------------------
  // PUSH THE CYCLE STEPS INTO THE VECTOR CONTAINER:
  // PUSH SEQUENCE = CYCLE SEQUENCE !
  main_cycle_steps.push_back(new Aufwecken);
  main_cycle_steps.push_back(new Vorschieben);
  main_cycle_steps.push_back(new Schneiden);
  main_cycle_steps.push_back(new Stirzel);
  main_cycle_steps.push_back(new Festklemmen);
  main_cycle_steps.push_back(new Startdruck);
  main_cycle_steps.push_back(new Spannen);
  main_cycle_steps.push_back(new Schweissen);
  main_cycle_steps.push_back(new Abkuehlen);
  main_cycle_steps.push_back(new Wippenhebel);
  main_cycle_steps.push_back(new Entspannen);
  main_cycle_steps.push_back(new Zurueckfahren);
  main_cycle_steps.push_back(new Pause);
  //------------------------------------------------
  // CONFIGURE THE STATE CONTROLLER:
  state_controller.set_no_of_steps(main_cycle_steps.size());
  //------------------------------------------------

  state_controller.set_step_mode();

  zyl_hauptluft.set(0); // Hauptluftventil nicht öffnen

  zyl_tool_niederhalter.set(1);

  Serial.println("EXIT SETUP");
}

// MAIN LOOPS ******************************************************************

// STEP MODE -------------------------------------------------------------------
void run_step_mode() {

  // IF MACHINE STATE IS "RUNNING", RUN CURRENT STEP:
  if (state_controller.machine_is_running()) {
    main_cycle_steps[state_controller.get_current_step()]->do_stuff();
  }

  // IF STEP IS COMPLETED SWITCH TO NEXT STEP:
  if (main_cycle_steps[state_controller.get_current_step()]->is_completed()) {
    state_controller.switch_to_next_step();
    reset_flag_of_current_step();
  }

  // RIG STOPS AFTER EVERY COMPLETED STEP:
  if (state_controller.step_switch_has_happend()) {
    state_controller.set_machine_stop();
  }
}

// AUTO MODE -------------------------------------------------------------------
void run_auto_mode() {
  // IF MACHINE STATE IS "RUNNING", RUN CURRENT STEP:
  if (state_controller.machine_is_running()) {
    main_cycle_steps[state_controller.get_current_step()]->do_stuff();
  }

  // IF STEP IS COMPLETED SWITCH TO NEXT STEP:
  if (main_cycle_steps[state_controller.get_current_step()]->is_completed()) {
    state_controller.switch_to_next_step();
    reset_flag_of_current_step();
  }

  // RESET "MACHINE STOPPED ERROR TIMEOUT" AFTER EVERY STEP:
  if (state_controller.step_switch_has_happend()) {
    timeout_machine_stopped.reset_time();
  }
}

// MONITOR ERRORS --------------------------------------------------------------
void monitor_strap_detectors() {
  // BANDSENSOREN ABFRAGEN:
  if (is_in_display_debug_mode) {
    return;
  }
  if (!bandsensor_oben.get_raw_button_state() || !bandsensor_unten.get_raw_button_state()) {
    state_controller.set_machine_stop();
    state_controller.set_error_mode();
    error_message = "KEIN BAND";
  }
}

void manage_timeout_actions() {
  // TIMEOUT 1
  if (timeout_count == 1) {
    reset_machine();
    state_controller.set_reset_mode();
  }
  // TIMEOUT 2
  else if (timeout_count == 2) {
    reset_machine();
    state_controller.set_reset_mode();
  }
  // STOP
  else if (timeout_count >= 3) {
    stop_machine();
    timeout_count = 0;
    error_message = "STOPPED";
    state_controller.set_error_mode();
  }
}

void monitor_timeout() {
  // Reset timeout if machine is not running:
  if (!state_controller.machine_is_running()) {
    timeout_machine_stopped.reset_time();
  }
  // Watch timeout if machine is running
  else if (timeout_machine_stopped.has_timed_out()) {
    timeout_count++;
    manage_timeout_actions();
  }
}

// RESET MODE ------------------------------------------------------------------
void run_reset_mode() {
  if (state_controller.run_after_reset_is_active()) {
    error_message = "RUN RESET " + String(timeout_count);
    state_controller.set_auto_mode();
    state_controller.set_machine_running();
  }
}

// MAIN LOOP -------------------------------------------------------------------
void loop() {

  // UPDATE DISPLAY:
  nextion_loop();

  // MONITOR PRESSURE:
  read_and_process_pressure();

  // CHECK IF STRAP IS AVAILABLE:
  monitor_strap_detectors();

  // MONITOR TIMEOUT:
  monitor_timeout();

  // RUN STEP MODE:
  if (state_controller.is_in_step_mode()) {
    state_controller.set_run_after_reset(false);
    run_step_mode();
  }
  // RUN AUTO MODE:
  else if (state_controller.is_in_auto_mode()) {
    state_controller.set_run_after_reset(true);
    run_auto_mode();
  }
  // RUN RESET MODE:
  else if (state_controller.is_in_reset_mode()) {
    run_reset_mode();
  }

  // RUN SPINNER:
  if (state_controller.machine_is_running()) {
    spinner_is_running = true;
  } else {
    spinner_is_running = false;
  }

  // // MEASURE CYCLE TIME
  // runtime = micros() - runtime_stopwatch;
  // Serial.println(runtime);
  // runtime_stopwatch = micros();
}
