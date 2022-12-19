
/*
 * *****************************************************************************
 * BXT_BX4_KPL
 * *****************************************************************************
 * Program to control a test rig
 * *****************************************************************************
 * Michael Wettstein
 * November 2022, Zürich
 * *****************************************************************************
 * TODO:
 * State Controller implementieren / Loop Ablauf von Sealless Rig kopieren
 * Variablen umbenennen
 * Alle Schalter und Taster debouncen
 * Timer durch insomniatimer ersetzen
 * Kommentare und Anmerkungen an Style Guide anpassen und vereinheitlichen
 * Compiler Warnungen anschauen
 * Reset button soll auch Zylinder abstellen
 * Insomnia library timeout für main cycle verwenden
 * Restpausenzeit direkt von insomnia library abfragen.
 * Bug beheben auf Page 2 wird die bandvorschubdauer angezeigt oben rechts...
 * ...beim Wechsel von Seite 3 auf 2
 * *****************************************************************************
 */

// #include <Arduino.h>
#include <ArduinoSTL.h> //       https://github.com/mike-matera/ArduinoSTL
#include <Controllino.h> //       PIO Controllino Library
#include <Cylinder.h> //       https://github.com/chischte/cylinder-library
#include <EEPROM_Counter.h> //       https://github.com/chischte/eeprom-counter-library
#include <Insomnia.h> //             https://github.com/chischte/insomnia-delay-library
#include <Nextion.h> //              PIO Nextion library
#include <SD.h> //              PIO Adafruit SD library

#include <cycle_step.h> //     blueprint of a cycle step
#include <state_controller.h> //     keeps track of machine states

// PRE-SETUP SECTION / PIN LAYOUT **********************************************

State_controller state_controller;

// KNOBS AND POTENTIOMETERS:

// SENSORS:
#define bandsensor_oben CONTROLLINO_A0
#define bandsensor_unten CONTROLLINO_A1
#define taster_startposition CONTROLLINO_A2
#define taster_endposition CONTROLLINO_A3
#define drucksensor CONTROLLINO_A7 // 0-10V = 0-12barg

// VALVES / MOTORS:
Cylinder einschaltventil(CONTROLLINO_D7);
Cylinder zyl_feder_abluft(CONTROLLINO_D1);
Cylinder zyl_feder_zuluft(CONTROLLINO_D0);
Cylinder zyl_klemmblock(CONTROLLINO_D2);
Cylinder zyl_wippenhebel(CONTROLLINO_D5);
Cylinder zyl_spanntaste(CONTROLLINO_D3);
Cylinder zyl_messer(CONTROLLINO_D6);
Cylinder zyl_schweisstaste(CONTROLLINO_D4);
Cylinder zyl_loescherblink(CONTROLLINO_D11);

Insomnia cycle_step_delay;
Insomnia nex_force_update_delay;
Insomnia nex_reset_button_timeout(10000); // pushtime to reset counter

// GLOBAL VARIABLES ------------------------------------------------------------
// bool (1/0 or true/false)
// byte (0-255)
// int   (-32,768 to 32,767) / unsigned int: 0 to 65,535
// long  (-2,147,483,648 to 2,147,483,647)
// float (6-7 Digits)

bool machine_running = false;
bool step_mode = true;
bool clearance_next_step = false;
bool band_vorhanden = false;
bool startposition_erreicht;
bool endposition_erreicht;

// byte Testzyklenzaehler;
byte cycle_step = 0;

unsigned long restpausenzeit;
unsigned long runtime;
unsigned long runtime_stopwatch;
unsigned long startfuelltimer;

// PUBLIC DRUCK UND KRAFT:
float pressure_float;
unsigned int force_int;

// SET UP EEPROM COUNTER:
enum eeprom_counter {
  startfuelldauer,
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
void reset_flag_of_current_step();
String get_main_cycle_display_string();
void display_text_in_field(String text, String text_field);
void send_to_nextion();

// CREATE VECTOR CONTAINER FOR THE CYCLE STEPS OBJECTS *************************

int Cycle_step::object_count = 0; // enable object counting
std::vector<Cycle_step *> main_cycle_steps;

// NON NEXTION FUNCTIONS *******************************************************

void reset_flag_of_current_step() { main_cycle_steps[state_controller.get_current_step()]->reset_flags(); }

// NEXTION VARIABLES -----------------------------------------------------------
int nex_current_page;

// NEXTION SWITCH STATES LIST --------------------------------------------------
// Every nextion switch button (dualstate) needs a switchstate variable to
// control switchtoggle. Every nextion button (momentary) needs a variable to
// prevent screen flickering.

bool nex_state_einschaltventil;
bool nex_state_zyl_feder_zuluft;
bool nex_state_zyl_feder_abluft;
bool nex_state_zyl_klemmblock;
bool nex_state_zyl_wippenhebel;
bool nex_state_zyl_spanntaste;
bool nex_state_zyl_messer;
bool nex_state_zyl_schweisstaste;
bool nex_state_machine_running;
bool nex_state_band_vorhanden = 1;
//***************************************************************************
bool nex_prev_step_mode = true;

byte nex_prev_cycle_step;
long nex_prev_cycles_in_a_row;
long nex_prev_long_cooldown_time;
long nex_prev_strap_eject_feed_time;
long nex_prev_shorttime_counter;
long nex_prev_longtime_counter;
unsigned int nex_prev_startfuelldauer;
unsigned int nex_prev_force_int;
unsigned int nex_prev_federdruck;
unsigned long nex_prev_restpausenzeit;
unsigned long button_push_stopwatch;

// NEXTION OBJECTS -------------------------------------------------------------

// PAGE 0:
NexPage nex_page0 = NexPage(0, 0, "page0");

// PAGE 1 - LEFT SIDE:
NexPage nex_page_1 = NexPage(1, 0, "page1");
NexButton nex_but_stepback = NexButton(1, 6, "b1");
NexButton nex_but_stepnxt = NexButton(1, 7, "b2");
NexButton nex_but_reset_cycle = NexButton(1, 5, "b0");
NexDSButton nex_switch_play_pause = NexDSButton(1, 2, "bt0");
NexDSButton nex_switch_mode = NexDSButton(1, 4, "bt1");

// PAGE 1 - RIGHT SIDE
NexDSButton nex_zyl_feder_zuluft = NexDSButton(1, 14, "bt5");
NexDSButton nex_zyl_feder_abluft = NexDSButton(1, 13, "bt4");
NexDSButton nex_zyl_klemmblock = NexDSButton(1, 12, "bt3");
NexButton nex_zyl_wippenhebel = NexButton(1, 11, "b5");
NexButton nex_mot_band_unten = NexButton(1, 10, "b4");
NexDSButton nex_zyl_messer = NexDSButton(1, 17, "b6");
NexButton nex_zyl_schweisstaste = NexButton(1, 8, "b3");
NexButton nex_einschaltventil = NexButton(1, 16, "bt6");

// PAGE 2 - LEFT SIDE:
NexPage nex_page_2 = NexPage(2, 0, "page2");
NexButton nex_but_slider_1_left = NexButton(2, 5, "b1");
NexButton nex_but_slider_1_right = NexButton(2, 6, "b2");

// PAGE 2 - RIGHT SIDE:
NexButton nex_but_reset_shorttime_counter = NexButton(2, 15, "b4");

// PAGE 3:
NexPage nex_page_3 = NexPage(3, 0, "page3");
NexButton nex_button_1_left = NexButton(3, 5, "b1");
NexButton nex_button_1_right = NexButton(3, 6, "b2");
NexButton nex_button_2_left = NexButton(3, 8, "b0");
NexButton nex_button_2_right = NexButton(3, 10, "b3");
NexButton nex_button_3_left = NexButton(3, 12, "b4");
NexButton nex_button_3_right = NexButton(3, 14, "b5");

char buffer[100] = {0}; // This is needed only if you are going to receive a
    // text from the display. You can remove it otherwise.

// NEXTION TOUCH EVENT LISTENERS -----------------------------------------------

NexTouch *nex_listen_list[] = {
    // PAGES
    &nex_page0, &nex_page_1, &nex_page_2, &nex_page_3,
    // PAGE 0 1 2:
    &nex_but_reset_shorttime_counter, &nex_but_stepback, &nex_but_stepnxt, &nex_but_reset_cycle, &nex_but_slider_1_left,
    &nex_but_slider_1_right, &nex_switch_play_pause, &nex_switch_mode, &nex_zyl_messer, &nex_zyl_klemmblock,
    &nex_zyl_feder_zuluft, &nex_zyl_feder_abluft, &nex_zyl_wippenhebel, &nex_mot_band_unten, &nex_zyl_schweisstaste,
    &nex_einschaltventil,
    // PAGE 3:
    &nex_button_1_left, &nex_button_1_right, &nex_button_2_left, &nex_button_2_right, &nex_button_3_left,
    &nex_button_3_right,
    // END OF DECLARATION
    NULL // String terminated
};
// NEXTION TOUCH EVENT FUNCTIONS -----------------------------------------------

// TOUCH EVENT FUNCTIONS PAGE 1 - LEFT SIDE ------------------------------------

void nex_switch_play_pause_push_callback(void *ptr) {
  machine_running = !machine_running;
  if (machine_running) {
    clearance_next_step = true;
  }
  nex_state_machine_running = !nex_state_machine_running;
}
void nex_switch_mode_push_callback(void *ptr) {
  state_controller.toggle_step_auto_mode();
  step_mode = state_controller.is_in_step_mode();
  Serial2.print("click bt1,1"); // CLICK BUTTON
  send_to_nextion();
}
void nex_but_stepback_push_callback(void *ptr) {
  state_controller.set_machine_stop();
  reset_flag_of_current_step();
  state_controller.set_step_mode();
  state_controller.switch_to_previous_step();
  reset_flag_of_current_step();
}
void nex_but_stepnxt_push_callback(void *ptr) {
  state_controller.set_machine_stop();
  reset_flag_of_current_step();
  state_controller.set_step_mode();
  state_controller.switch_to_next_step();
  reset_flag_of_current_step();
}
void nex_but_reset_cycle_push_callback(void *ptr) {
  cycle_step = 0;
  step_mode = true;
}

// TOUCH EVENT FUNCTIONS PAGE 1 - RIGHT SIDE -----------------------------------

void nex_zyl_feder_zuluft_push_callback(void *ptr) {
  zyl_feder_zuluft.toggle();
  nex_state_zyl_feder_zuluft = !nex_state_zyl_feder_zuluft;
}

void nex_zyl_feder_abluft_push_callback(void *ptr) {
  zyl_feder_abluft.toggle();
  nex_state_zyl_feder_abluft = !nex_state_zyl_feder_abluft;
}

void nex_zyl_klemmblock_push_callback(void *ptr) {
  zyl_klemmblock.toggle();
  nex_state_zyl_klemmblock = !nex_state_zyl_klemmblock;
}

void nex_zyl_wippenhebel_push_callback(void *ptr) { zyl_wippenhebel.set(1); }

void nex_zyl_wippenhebel_pop_callback(void *ptr) { zyl_wippenhebel.set(0); }

void nex_mot_band_unten_push_callback(void *ptr) { zyl_spanntaste.set(1); }
void nex_mot_band_unten_pop_callback(void *ptr) { zyl_spanntaste.set(0); }
void nex_zyl_schweisstaste_push_callback(void *ptr) { zyl_schweisstaste.set(1); }
void nex_zyl_schweisstaste_pop_callback(void *ptr) { zyl_schweisstaste.set(0); }
void nex_zyl_messer_push_callback(void *ptr) { zyl_messer.set(1); }
void nex_zyl_messer_pop_callback(void *ptr) { zyl_messer.set(0); }
void nex_einschaltventil_push_callback(void *ptr) {
  einschaltventil.toggle();
  nex_state_einschaltventil = !nex_state_einschaltventil;
}

// TOUCH EVENT FUNCTIONS PAGE 2 - LEFT SIDE ------------------------------------

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

void nex_but_slider_1_left_push_callback(void *ptr) { decrease_slider_value(startfuelldauer, 0, 100); }

void nex_but_slider_1_right_push_callback(void *ptr) { increase_slider_value(startfuelldauer, 7000, 100); }

// TOUCH EVENT FUNCTIONS PAGE 2 - RIGHT SIDE -----------------------------------

void nex_but_reset_shorttime_counter_push_callback(void *ptr) {
  eeprom_counter.set_value(shorttime_counter, 0);
  // RESET LONGTIME COUNTER IF RESET BUTTON IS PRESSED LONG ENOUGH:
  // ACTIVATE TIMEOUT TO RESET LONGTIME COUNTER:
  nex_reset_button_timeout.reset_time();
  nex_reset_button_timeout.set_flag_activated(1);
}

void nex_but_reset_shorttime_counter_pop_callback(void *ptr) { nex_reset_button_timeout.set_flag_activated(0); }
// TOUCH EVENT FUNCTIONS PAGE 3 ------------------------------------------------

void nex_button_1_left_push_callback(void *ptr) { decrease_slider_value(cycles_in_a_row, 0, 1); }

void nex_button_1_right_push_callback(void *ptr) { increase_slider_value(cycles_in_a_row, 10, 1); }

void nex_button_2_left_push_callback(void *ptr) { decrease_slider_value(long_cooldown_time, 0, 10); }

void nex_button_2_right_push_callback(void *ptr) { increase_slider_value(long_cooldown_time, 600, 10); }

void nex_button_3_left_push_callback(void *ptr) { decrease_slider_value(strap_eject_feed_time, 0, 1); }
void nex_button_3_right_push_callback(void *ptr) { increase_slider_value(strap_eject_feed_time, 20, 1); }

// TOUCH EVENT FUNCTIONS PAGE CHANGES ------------------------------------------

void nex_page0_push_callback(void *ptr) { nex_current_page = 0; }

void nex_page_1_push_callback(void *ptr) {
  nex_current_page = 1;

  // REFRESH BUTTON STATES:
  nex_prev_cycle_step = 1;
  nex_prev_step_mode = true;

  nex_state_zyl_feder_zuluft = 0;
  nex_state_zyl_feder_abluft = 1; // INVERTED VALVE LOGIC
  nex_state_zyl_klemmblock = 0;
  nex_state_zyl_wippenhebel = 0;
  nex_state_zyl_spanntaste = 0;
  nex_state_zyl_messer = 0;
  nex_state_zyl_schweisstaste = 0;
  nex_state_machine_running = 0;
  nex_state_band_vorhanden = !band_vorhanden;
  nex_state_einschaltventil = 0;
}

void nex_page_2_push_callback(void *ptr) {
  nex_current_page = 2;
  // REFRESH BUTTON STATES:
  nex_prev_startfuelldauer = 0;
  nex_prev_shorttime_counter = 0;
  nex_prev_longtime_counter = 0;
  nex_prev_force_int = 0;
  nex_prev_federdruck = 10000;
}

void nex_page_3_push_callback(void *ptr) {
  nex_current_page = 3;
  // REFRESH BUTTON STATES:
  nex_prev_cycles_in_a_row = 0;
  nex_prev_long_cooldown_time = 0;
  nex_prev_strap_eject_feed_time = 0;
}
// END OF NEXTION TOUCH EVENT FUNCTIONS ****************************************

// NEXTION SETUP ***************************************************************

void nextion_setup() { // START NEXTION SETUP
  Serial2.begin(9600); // Start serial comunication at baud=9600

  // REGISTER EVENT CALLBACK FUNCTIONS -----------------------------------------

  // PAGE 0 1 2:
  nex_page0.attachPush(nex_page0_push_callback);
  nex_page_1.attachPush(nex_page_1_push_callback);
  nex_page_2.attachPush(nex_page_2_push_callback);
  nex_page_3.attachPush(nex_page_3_push_callback);
  nex_but_stepback.attachPush(nex_but_stepback_push_callback);
  nex_but_stepnxt.attachPush(nex_but_stepnxt_push_callback);
  nex_zyl_klemmblock.attachPush(nex_zyl_klemmblock_push_callback);
  nex_but_reset_cycle.attachPush(nex_but_reset_cycle_push_callback);
  nex_but_slider_1_left.attachPush(nex_but_slider_1_left_push_callback);
  nex_but_slider_1_right.attachPush(nex_but_slider_1_right_push_callback);
  nex_but_stepback.attachPush(nex_but_stepback_push_callback);
  nex_but_stepnxt.attachPush(nex_but_stepnxt_push_callback);
  nex_switch_mode.attachPush(nex_switch_mode_push_callback);
  nex_switch_play_pause.attachPush(nex_switch_play_pause_push_callback);
  nex_zyl_klemmblock.attachPush(nex_zyl_klemmblock_push_callback);
  nex_zyl_feder_zuluft.attachPush(nex_zyl_feder_zuluft_push_callback);
  nex_zyl_feder_abluft.attachPush(nex_zyl_feder_abluft_push_callback);
  nex_einschaltventil.attachPush(nex_einschaltventil_push_callback);
  // PAGE 3:
  nex_button_1_left.attachPush(nex_button_1_left_push_callback);
  nex_button_1_right.attachPush(nex_button_1_right_push_callback);
  nex_button_2_left.attachPush(nex_button_2_left_push_callback);
  nex_button_2_right.attachPush(nex_button_2_right_push_callback);
  nex_button_3_left.attachPush(nex_button_3_left_push_callback);
  nex_button_3_right.attachPush(nex_button_3_right_push_callback);

  //*****PUSH+POP:
  nex_zyl_wippenhebel.attachPush(nex_zyl_wippenhebel_push_callback);
  nex_zyl_wippenhebel.attachPop(nex_zyl_wippenhebel_pop_callback);
  nex_mot_band_unten.attachPush(nex_mot_band_unten_push_callback);
  nex_mot_band_unten.attachPop(nex_mot_band_unten_pop_callback);
  nex_zyl_schweisstaste.attachPush(nex_zyl_schweisstaste_push_callback);
  nex_zyl_schweisstaste.attachPop(nex_zyl_schweisstaste_pop_callback);
  nex_zyl_messer.attachPush(nex_zyl_messer_push_callback);
  nex_zyl_messer.attachPop(nex_zyl_messer_pop_callback);
  nex_but_reset_shorttime_counter.attachPush(nex_but_reset_shorttime_counter_push_callback);
  nex_but_reset_shorttime_counter.attachPop(nex_but_reset_shorttime_counter_pop_callback);

  // ---------------------------------------------------------------------------

  delay(3000);
  sendCommand("page 1"); // SWITCH NEXTION TO PAGE X
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
  Serial2.print("t4");
  Serial2.print(".txt=");
  Serial2.print("\"");
  Serial2.print(text);
  Serial2.print("\"");
  send_to_nextion();
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

// NEXTION TOUCH EVENT FUNCTIONS ***********************************************

// TOUCH EVENT FUNCTIONS PAGE 1 - LEFT SIDE ------------------------------------

// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk
// sdhajkflsdhfklsdfhsdakfhdsjkflhsdafjklshdajk

// NEXTION DISPLAY LOOPS *******************************************************

// DISPLAY LOOP PAGE 1 LEFT SIDE: ----------------------------------------------

String get_main_cycle_display_string() {
  int current_step = state_controller.get_current_step();
  String display_text_cycle_name = main_cycle_steps[current_step]->get_display_text();
  return display_text_cycle_name;
}

void update_main_cycle_name() {
  if (nex_prev_cycle_step != state_controller.get_current_step()) {
    String number = String(state_controller.get_current_step() + 1);
    String name = get_main_cycle_display_string();
    Serial.println(number + " " + name);
    display_text_in_field(number + " " + name, "t0");
    nex_prev_cycle_step = state_controller.get_current_step();
  }
}

void update_cycle_name() {
  if (state_controller.is_in_step_mode() || state_controller.is_in_auto_mode()) {
    update_main_cycle_name();
  }
}

void display_loop_page_1_left_side() {

  update_cycle_name();

  // UPDATE SWITCHSTATE "PLAY"/"PAUSE"
  if (nex_state_machine_running != machine_running) {
    Serial2.print("click bt0,1"); // CLICK BUTTON
    send_to_nextion();
    nex_state_machine_running = !nex_state_machine_running;
  }

  // UPDATE SWITCHSTATE "STEP"/"AUTO"-MODE
  if (step_mode != nex_prev_step_mode) {
    Serial2.print("click bt1,1"); // CLICK BUTTON
    send_to_nextion();
    nex_prev_step_mode = step_mode;
  }

  // DISPLAY IF NO STRAP DETECTED
  if (nex_state_band_vorhanden != band_vorhanden) {
    if (!band_vorhanden) {
      Serial2.print("t4.txt=");
      Serial2.print("\"");
      Serial2.print("BAND LEER!");
      Serial2.print("\"");
      send_to_nextion();
    } else {
      Serial2.print("t4.txt=");
      Serial2.print("\"");
      Serial2.print(""); // ERASE TEXT
      Serial2.print("\"");
      send_to_nextion();
    }
    nex_state_band_vorhanden = band_vorhanden;
  }

  if (band_vorhanden && (nex_prev_restpausenzeit != restpausenzeit)) {
    if (restpausenzeit > 0 && restpausenzeit < 1000) {
      Serial2.print("t4.txt=");
      Serial2.print("\"");
      Serial2.print("PAUSE: ");
      Serial2.print(restpausenzeit); // SHOW REMAINING PAUSE TIME
      Serial2.print("s");
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_restpausenzeit = restpausenzeit;
    } else {
      Serial2.print("t4.txt=");
      Serial2.print("\"");
      Serial2.print(""); // ERASE TEXT
      Serial2.print("\"");
      send_to_nextion();
    }
  }
}

// DISPLAY LOOP PAGE 1 RIGHT SIDE: ---------------------------------------------

void display_loop_page_1_right_side() {

  // UPDATE SWITCHBUTTON (dual state):
  if (zyl_feder_zuluft.get_state() != nex_state_zyl_feder_zuluft) {
    Serial2.print("click bt5,1"); // CLICK BUTTON
    send_to_nextion();
    nex_state_zyl_feder_zuluft = !nex_state_zyl_feder_zuluft;
  }
  // UPDATE SWITCHBUTTON (dual state):
  if (zyl_feder_abluft.get_state() != nex_state_zyl_feder_abluft) {
    Serial2.print("click bt4,1"); // CLICK BUTTON
    send_to_nextion();
    nex_state_zyl_feder_abluft = !nex_state_zyl_feder_abluft;
  }
  // UPDATE SWITCHBUTTON (dual state):
  if (zyl_klemmblock.get_state() != nex_state_zyl_klemmblock) {
    Serial2.print("click bt3,1"); // CLICK BUTTON
    send_to_nextion();
    nex_state_zyl_klemmblock = !nex_state_zyl_klemmblock;
  }

  // UPDATE BUTTON (momentary)
  if (zyl_wippenhebel.get_state() != nex_state_zyl_wippenhebel) {
    Serial2.print("click b5,");
    Serial2.print(zyl_wippenhebel.get_state()); // PUSH OR RELEASE BUTTON
    send_to_nextion();
    nex_state_zyl_wippenhebel = zyl_wippenhebel.get_state();
  }

  // UPDATE BUTTON (momentary)
  if (zyl_spanntaste.get_state() != nex_state_zyl_spanntaste) {
    Serial2.print("click b4,");
    Serial2.print(zyl_spanntaste.get_state()); // PUSH OR RELEASE BUTTON
    send_to_nextion();
    nex_state_zyl_spanntaste = zyl_spanntaste.get_state();
  }

  // UPDATE SWITCHBUTTON (dual state):
  if (zyl_messer.get_state() != nex_state_zyl_messer) {
    Serial2.print("click b6,");
    Serial2.print(zyl_messer.get_state()); // PUSH OR RELEASE BUTTON
    send_to_nextion();
    nex_state_zyl_messer = zyl_messer.get_state();
  }
  // UPDATE BUTTON (momentary)
  if (zyl_schweisstaste.get_state() != nex_state_zyl_schweisstaste) {
    Serial2.print("click b3,");
    Serial2.print(zyl_schweisstaste.get_state()); // PUSH OR RELEASE
        // BUTTON
    send_to_nextion();
    nex_state_zyl_schweisstaste = zyl_schweisstaste.get_state();
  }

  // UPDATE BUTTON (momentary button used as dual state toggle)
  if (einschaltventil.get_state() != nex_state_einschaltventil) {
    Serial2.print("click bt6,1"); // CLICK BUTTON
    send_to_nextion();
    nex_state_einschaltventil = einschaltventil.get_state();
  }
}

// DIPLAY LOOP PAGE 2 LEFT SIDE: -----------------------------------------------

void update_force_display() {
  if (nex_prev_force_int != force_int) {
    display_value_in_field(force_int, "h1");
    String suffixed_value = add_suffix_to_value(force_int, "N");
    display_text_in_field(suffixed_value, "t6");
    nex_prev_force_int = force_int;
  }
}

void update_pressure_display() {
  Serial2.print("t8.txt=");
  Serial2.print("\"");
  Serial2.print(pressure_float, 1);
  Serial2.print(" bar");
  Serial2.print("\"");
  send_to_nextion();
  nex_prev_federdruck = pressure_float;
}

void update_startfuelldauer() {
  if (nex_prev_startfuelldauer != eeprom_counter.get_value(startfuelldauer)) {
    int value = eeprom_counter.get_value(startfuelldauer);
    String display_string = add_suffix_to_value(value, "ms");
    display_text_in_field(display_string, "t4");
    nex_prev_startfuelldauer = eeprom_counter.get_value(startfuelldauer);
  }
}

void display_loop_page_2_left_side() {

  update_startfuelldauer();

  if (nex_force_update_delay.delay_time_is_up(200)) {
    update_force_display();
    update_pressure_display();
  }
}

// DIPLAY LOOP PAGE 2 RIGHT SIDE: ----------------------------------------------

void update_upper_counter_value() {
  if (nex_prev_longtime_counter != eeprom_counter.get_value(longtime_counter)) {
    display_text_in_field(String(eeprom_counter.get_value(longtime_counter)), "t10");
    nex_prev_longtime_counter = eeprom_counter.get_value(longtime_counter);
  }
}

void update_lower_counter_value() {
  // UPDATE LOWER COUNTER:
  if (nex_prev_shorttime_counter != eeprom_counter.get_value(shorttime_counter)) {
    display_text_in_field(String(eeprom_counter.get_value(shorttime_counter)), "t12");
    nex_prev_shorttime_counter = eeprom_counter.get_value(shorttime_counter);
  }
}

void reset_lower_counter_value() {
  if (nex_reset_button_timeout.is_marked_activated()) {
    if (nex_reset_button_timeout.has_timed_out()) {
      eeprom_counter.set_value(longtime_counter, 0);
    }
  }
}

void display_loop_page_2_right_side() {
  update_upper_counter_value();
  update_lower_counter_value();
  reset_lower_counter_value();
}

// DIPLAY LOOP PAGE 3: ---------------------------------------------------------

void update_number_of_cycles() {
  if (nex_prev_cycles_in_a_row != eeprom_counter.get_value(cycles_in_a_row)) {
    String text = String(eeprom_counter.get_value(cycles_in_a_row));
    display_text_in_field(text, "t4");
    nex_prev_cycles_in_a_row = eeprom_counter.get_value(cycles_in_a_row);
  }
}

void update_cooldown_time() {
  if (nex_prev_long_cooldown_time != eeprom_counter.get_value(long_cooldown_time)) {
    long value = eeprom_counter.get_value(long_cooldown_time);
    String text = add_suffix_to_value(value, "s");
    display_text_in_field(text, "t5");
    nex_prev_long_cooldown_time = eeprom_counter.get_value(long_cooldown_time);
  }
}

void update_strap_feed_time() {
  if (nex_prev_strap_eject_feed_time != eeprom_counter.get_value(strap_eject_feed_time)) {
    long value = eeprom_counter.get_value(strap_eject_feed_time);
    String text = add_suffix_to_value(value, "s");
    display_text_in_field(text, "t7");
    nex_prev_strap_eject_feed_time = eeprom_counter.get_value(strap_eject_feed_time);
  }
}

void display_loop_page_3() {
  update_number_of_cycles();
  update_cooldown_time();
  update_strap_feed_time();
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
    display_loop_page_2_left_side();
    display_loop_page_2_right_side();
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
  return analogRead(drucksensor) / 27.778; //[bar]
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
  static float min_difference = 0.05; // [bar]

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

  // CALCULATE FORCE:
  int force = pressure * 1472.6; // 1bar  => 1472.6N (Dauertest BXT 3-32 Zylinderkraft.xlsx)

  // SET LAST DIGIT ZERO
  force = force / 10;
  force = force * 10;

  return force;
}

void read_and_process_pressure() {

  pressure_float = get_pressure_from_sensor(); //[bar]
  pressure_float = smoothe_measurement(pressure_float); //[bar]
  pressure_float = calm_measurement(pressure_float);

  pressure_float = pressure_float; //[bar]
  force_int = convert_pressure_to_force(pressure_float); // [N]
}

void monitor_strap_detectors() {

  // BANDSENSOREN ABFRAGEN:
  if (digitalRead(bandsensor_oben) && digitalRead(bandsensor_unten)) {
    band_vorhanden = true;
  } else {
    band_vorhanden = false;
    state_controller.set_machine_stop();
  }

  // START- UND ENDPOSITIONSSCHALTER ABFRAGEN:
  startposition_erreicht = digitalRead(taster_startposition);
  endposition_erreicht = digitalRead(taster_endposition);
}

// CREATE CYCLE STEP CLASSES ***************************************************
// -----------------------------------------------------------------------------
class Aufwecken_ : public Cycle_step {
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

  void do_initial_stuff() { feed_time = eeprom_counter.get_value(strap_eject_feed_time) * 1000; };
  void do_loop_stuff() {
    zyl_spanntaste.stroke(feed_time, 300);
    if (zyl_spanntaste.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Schneiden : public Cycle_step {
  String get_display_text() { return "SCHNEIDEN"; }

  void do_initial_stuff(){};
  void do_loop_stuff() {
    zyl_messer.stroke(1500, 500);

    if (zyl_messer.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Festklemmen : public Cycle_step {
  String get_display_text() { return "FESTKLEMMEN"; }

  void do_initial_stuff() {
    zyl_klemmblock.set(1);
    cycle_step_delay.set_unstarted();
  };
  void do_loop_stuff() {
    if (cycle_step_delay.delay_time_is_up(400)) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Startdruck : public Cycle_step {
  String get_display_text() { return "STARTDRUCK"; }

  void do_initial_stuff() {
    cycle_step_delay.set_unstarted();
    zyl_feder_zuluft.set(1); // 1=füllen 0=geschlossen
    zyl_feder_abluft.set(1); // 1=geschlossen 0=entlüften
  };

  void do_loop_stuff() {
    if (cycle_step_delay.delay_time_is_up(eeprom_counter.get_value(startfuelldauer))) {
      zyl_feder_zuluft.set(0); // 1=füllen 0=geschlossen
      set_loop_completed();
    };
  };
};
// -----------------------------------------------------------------------------
class Spannen : public Cycle_step {
  String get_display_text() { return "SPANNEN"; }

  void do_initial_stuff() {
    zyl_spanntaste.set(1); // Spanntaste betätigen
    cycle_step_delay.set_unstarted();
  };
  void do_loop_stuff() {
    if (endposition_erreicht) {
      if (cycle_step_delay.delay_time_is_up(400)) {
        set_loop_completed();
      }
    };
  };
};
// -----------------------------------------------------------------------------
class Schweissen : public Cycle_step {
  String get_display_text() { return "SCHWEISSEN"; }

  void do_initial_stuff() {
    zyl_spanntaste.set(0); // Spanntaste lösen
  };

  void do_loop_stuff() {
    zyl_schweisstaste.stroke(1000, 2000); // Schweisstaste betätigen

    if (zyl_schweisstaste.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
// Abkühlen und Druck abbauen
class Abkuehlen : public Cycle_step {
  String get_display_text() { return "ABKUELHEN"; }

  void do_initial_stuff() {
    cycle_step_delay.set_unstarted();
    zyl_feder_zuluft.set(0); // 1=füllen 0=geschlossen
    zyl_feder_abluft.set(0); // 1=geschlossen 0=entlüften
  };
  void do_loop_stuff() {
    if (pressure_float < 0.1) // warten bis der Druck ist abgebaut
    {
      if (cycle_step_delay.delay_time_is_up(4000)) { // Restluft kann entweichen
        set_loop_completed();
      }
    }
  };
};
// -----------------------------------------------------------------------------
class Entspannen : public Cycle_step {
  String get_display_text() { return "ENTSPANNEN"; }

  void do_initial_stuff() {
    cycle_step_delay.set_unstarted();
    zyl_klemmblock.set(0);
  };
  void do_loop_stuff() {
    if (cycle_step_delay.delay_time_is_up(500)) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Wippenhebel : public Cycle_step {
  String get_display_text() { return "WIPPENHEBEL"; }

  void do_initial_stuff(){};
  void do_loop_stuff() {
    zyl_wippenhebel.stroke(1500, 1000); //(Ausfahrzeit,Einfahrzeit)

    if (zyl_wippenhebel.stroke_completed()) {
      set_loop_completed();
    }
  };
};
// -----------------------------------------------------------------------------
class Zurueckfahren : public Cycle_step {
  String get_display_text() { return "ZURUECKFAHREN"; }

  void do_initial_stuff() {
    zyl_feder_zuluft.set(1); // 1=füllen 0=geschlossen
    zyl_feder_abluft.set(0); // 1=geschlossen 0=entlüften
    cycle_step_delay.set_unstarted();
  };
  void do_loop_stuff() {
    if (startposition_erreicht) {
      zyl_feder_zuluft.set(0); // 1=füllen 0=geschlossen
      zyl_feder_abluft.set(0); // 1=geschlossen 0=entlüften
      if (pressure_float < 0.1) // warten bis der Druck abgebaut ist
      {
        if (cycle_step_delay.delay_time_is_up(500)) {
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
  unsigned long abkuehldauer;

  void do_initial_stuff() {
    testZyklenZaehler++;
    abkuehldauer = eeprom_counter.get_value(long_cooldown_time) * 1000;
  };

  void do_loop_stuff() {
    if (testZyklenZaehler == eeprom_counter.get_value(cycles_in_a_row)) {
      if (cycle_step_delay.delay_time_is_up(abkuehldauer)) {
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

void setup() {
  eeprom_counter.setup(eeprom_min_address, eeprom_max_address, number_of_eeprom_values);
  Serial.begin(115200); // start serial connection

  nextion_setup();

  pinMode(bandsensor_oben, INPUT);
  pinMode(bandsensor_unten, INPUT);
  pinMode(taster_startposition, INPUT);
  pinMode(taster_endposition, INPUT);
  pinMode(drucksensor, INPUT);

  delay(2000);

  //------------------------------------------------
  // PUSH THE CYCLE STEPS INTO THE VECTOR CONTAINER:
  // PUSH SEQUENCE = CYCLE SEQUENCE !
  main_cycle_steps.push_back(new Aufwecken_);
  main_cycle_steps.push_back(new Vorschieben);
  main_cycle_steps.push_back(new Schneiden);
  main_cycle_steps.push_back(new Festklemmen);
  main_cycle_steps.push_back(new Startdruck);
  main_cycle_steps.push_back(new Spannen);
  main_cycle_steps.push_back(new Schweissen);
  main_cycle_steps.push_back(new Abkuehlen);
  main_cycle_steps.push_back(new Entspannen);
  main_cycle_steps.push_back(new Wippenhebel);
  main_cycle_steps.push_back(new Zurueckfahren);
  main_cycle_steps.push_back(new Pause);
  //------------------------------------------------
  // CONFIGURE THE STATE CONTROLLER:
  int no_of_main_cycle_steps = main_cycle_steps.size();
  state_controller.set_no_of_steps(no_of_main_cycle_steps);
  //------------------------------------------------

  einschaltventil.set(1); //ÖFFNET DAS HAUPTLUFTVENTIL

  Serial.println("EXIT SETUP");
}

// MAIN LOOP
// *******************************************************************

void run_step_or_auto_mode() {

  // IF STEP IS COMPLETED SWITCH TO NEXT STEP:
  if (main_cycle_steps[state_controller.get_current_step()]->is_completed()) {
    state_controller.switch_to_next_step();
    reset_flag_of_current_step();
  }

  // IN STEP MODE, THE RIG STOPS AFTER EVERY COMPLETED STEP:
  if (state_controller.step_switch_has_happend()) {
    if (state_controller.is_in_step_mode()) {
      state_controller.set_machine_stop();
    }
    // IF MACHINE STATE IS RUNNING IN AUTO MODE,
    // THE "MACHINE STOPPED ERROR TIMEOUT" RESETS AFTER EVERY STEP:
    if (state_controller.is_in_auto_mode() && state_controller.machine_is_running()) {
      // machine_stopped_error_timeout.reset_time();
    }
  }

  // IF MACHINE STATE IS "RUNNING", RUN CURRENT STEP:
  if (state_controller.machine_is_running()) {
    main_cycle_steps[state_controller.get_current_step()]->do_stuff();
  }

  // MEASURE AND DISPLAY PRESSURE
  if (!state_controller.is_in_error_mode()) {
    // measure_and_display_max_force();
  }
}

void loop() {

  read_and_process_pressure();

  // UPDATE DISPLAY:
  nextion_loop();

  // CHECK IF STRAP IS AVAILABLE
  monitor_strap_detectors();

  // MONITOR EMERGENCY SIGNAL:
  // monitor_emergency_signal();

  // DO NOT WATCH TIMEOUTS IF MACHINE IS NOT RUNNING (PAUSE):
  if (!state_controller.machine_is_running()) {
    // machine_stopped_error_timeout.reset_time();
    // bandsensor_timeout.reset_time();
  }

  // MONITOR ERRORS ONLY WHEN RIG IS RUNNING IN AUTO MODE:
  if (state_controller.machine_is_running() && state_controller.is_in_auto_mode()) {
    // monitor_error_timeouts();
    // monitor_temperature_error();
  }

  // RUN STEP OR AUTO MODE:
  if (state_controller.is_in_step_mode() || state_controller.is_in_auto_mode()) {
    run_step_or_auto_mode();
  }

  // RUN RESET IF RESET IS ACTIVATED:
  if (state_controller.reset_mode_is_active()) {
    // reset_machine();
    // stroke_wippenhebel();
    state_controller.set_reset_mode(0);

    if (state_controller.run_after_reset_is_active()) {
      state_controller.set_auto_mode();
      state_controller.set_machine_running();
    } else {
      state_controller.set_step_mode();
    }
  }
}

// runtime = millis() - runtime_stopwatch;
// Serial.println(runtime);
// runtime_stopwatch = millis();
