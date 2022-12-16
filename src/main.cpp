
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
 * Cyclesteps zu Objekten umschreiben
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
#include <ArduinoSTL.h>  //       https://github.com/mike-matera/ArduinoSTL
#include <Controllino.h> //       PIO Controllino Library
#include <Cylinder.h>    //       https://github.com/chischte/cylinder-library
#include <EEPROM_Counter.h> //       https://github.com/chischte/eeprom-counter-library
#include <Insomnia.h> //             https://github.com/chischte/insomnia-delay-library
#include <Nextion.h> //              PIO Nextion library
#include <SD.h>      //              PIO Adafruit SD library

#include <cycle_step.h>       //     blueprint of a cycle step
#include <state_controller.h> //     keeps track of machine states

//*****************************************************************************
// PRE-SETUP SECTION / PIN LAYOUT
//*****************************************************************************

State_controller state_controller;

//*****************************************************************************
// DEFINE NAMES AND SEQUENCE OF STEPS FOR THE MAIN CYCLE:
//*****************************************************************************
// enum mainCycleSteps {
//   AUFWECKEN,
//   VORSCHIEBEN,
//   SCHNEIDEN,
//   FESTKLEMMEN,
//   STARTDRUCK,
//   SPANNEN,
//   SCHWEISSEN,
//   ABKUEHLEN,
//   ENTSPANNEN,
//   WIPPENHEBEL,
//   ZURUECKFAHREN,
//   PAUSE,
//   endOfMainCycleEnum
// };

// int numberOfMainCycleSteps = endOfMainCycleEnum;
// // DEFINE NAMES TO DISPLAY ON THE TOUCH SCREEN:
// String cycle_name[] = {"AUFWECKEN",   "VORSCHIEBEN",   "SCHNEIDEN",
//                        "FESTKLEMMEN", "STARTDRUCK",    "SPANNEN",
//                        "SCHWEISSEN",  "ABKUELHEN",     "ENTSPANNEN",
//                        "WIPPENHEBEL", "ZURUECKFAHREN", "PAUSE"};

// KNOBS AND POTENTIOMETERS:
#define start_button CONTROLLINO_A6
#define stop_button CONTROLLINO_A5
#define green_light CONTROLLINO_D9
#define red_light CONTROLLINO_D8

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

Insomnia errorBlinkTimer;
Insomnia cycle_step_delay;

//*****************************************************************************
// DECLARATION OF VARIABLES / DATA TYPES
//*****************************************************************************
// bool (1/0 or true/false)
// byte (0-255)
// int   (-32,768 to 32,767) / unsigned int: 0 to 65,535
// long  (-2,147,483,648 to 2,147,483,647)
// float (6-7 Digits)
//*****************************************************************************
bool machine_running = false;
bool step_mode = true;
bool clearance_next_step = false;
bool error_blink = false;
bool band_vorhanden = false;
bool startposition_erreicht;
bool endposition_erreicht;
bool startfuellung_running = false;

// byte Testzyklenzaehler;
byte cycle_step = 0;

int calmcounter;
unsigned long timer_next_step;

unsigned int federdruck_beruhigt;
unsigned int prev_federdruck_beruhigt;

unsigned long restpausenzeit;
unsigned long timer_error_blink;
unsigned long runtime;
unsigned long runtime_stopwatch;
unsigned long startfuelltimer;
unsigned long prev_time;
long bandVorschubDauer;
long calmcountersum;

// DRUCKRECHNUNG:
float federdruck_float;
float federdruck_smoothed;
unsigned long federdruck_mbar;
unsigned int federdruck_mbar_int;

// KRAFTRECHNUNG:
float federkraft;
unsigned long federkraft_smoothed;
unsigned int federkraft_int;

// SET UP EEPROM COUNTER:
enum eepromCounter {
  startfuelldauer,
  shorttimeCounter,
  longtimeCounter,
  cyclesInARow,
  longCooldownTime,
  strapEjectFeedTime,
  endOfEepromEnum
};
int numberOfEepromValues = endOfEepromEnum;
int eepromMinAddress = 0;
int eepromMaxAddress = 4095;
EEPROM_Counter eepromCounter;

// DECLARE FUNCTIONS IF NEEDED FOR THE COMPILER: *******************************
void reset_flag_of_current_step();
String get_main_cycle_display_string();
void display_text_in_field(String text, String textField);
void send_to_nextion();

// CREATE VECTOR CONTAINER FOR THE CYCLE STEPS OBJECTS *************************

int Cycle_step::object_count = 0; // enable object counting
std::vector<Cycle_step *> main_cycle_steps;

// NON NEXTION FUNCTIONS *******************************************************

void reset_flag_of_current_step() {
  main_cycle_steps[state_controller.get_current_step()]->reset_flags();
}

//***************************************************************************
// DECLARATION OF VARIABLES
//***************************************************************************
int nex_current_page;

//***************************************************************************
// NEXTION SWITCH STATES LIST
// Every nextion switch button (dualstate) needs a switchstate variable to
// control switchtoggle Nextion buttons(momentary) need a variable too, to
// prevent screen flickering

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
bool stopwatch_running;
bool reset_stopwatch_active;

byte nex_prev_cycle_step;
long nexPrevCyclesInARow;
long nexPrevLongCooldownTime;
long nexPrevStrapEjectFeedTime;
long nex_prev_shorttime_counter;
long nex_prev_longtime_counter;
unsigned int nex_prev_startfuelldauer;
unsigned int nex_prev_federkraft_int;
unsigned int nex_prev_federdruck;
unsigned long nex_prev_restpausenzeit;
unsigned long button_push_stopwatch;
unsigned long counter_reset_stopwatch;
unsigned long nex_update_timer;
//***************************************************************************
//***************************************************************************
// DECLARATION OF OBJECTS TO BE READ FROM NEXTION
//***************************************************************************

// PAGE 0:
NexPage nex_page0 = NexPage(0, 0, "page0");

// PAGE 1 - LEFT SIDE:
NexPage nex_page1 = NexPage(1, 0, "page1");
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
NexPage nex_page2 = NexPage(2, 0, "page2");
NexButton nex_but_slider1_left = NexButton(2, 5, "b1");
NexButton nex_but_slider1_right = NexButton(2, 6, "b2");

// PAGE 2 - RIGHT SIDE:
NexButton nex_but_reset_shorttime_counter = NexButton(2, 15, "b4");

// PAGE 3:
NexPage nex_page3 = NexPage(3, 0, "page3");
NexButton nexButton1Left = NexButton(3, 5, "b1");
NexButton nexButton1Right = NexButton(3, 6, "b2");
NexButton nexButton2Left = NexButton(3, 8, "b0");
NexButton nexButton2Right = NexButton(3, 10, "b3");
NexButton nexButton3Left = NexButton(3, 12, "b4");
NexButton nexButton3Right = NexButton(3, 14, "b5");

//***************************************************************************
// END OF OBJECT DECLARATION
//***************************************************************************

char buffer[100] = {0}; // This is needed only if you are going to receive a
                        // text from the display. You can remove it otherwise.

//***************************************************************************
// TOUCH EVENT LIST //DECLARATION OF TOUCH EVENTS TO BE MONITORED
//***************************************************************************
NexTouch *nex_listen_list[] = {
    &nex_page0, &nex_page1, &nex_page2, &nex_page3,
    // PAGE 0 1 2:
    &nex_but_reset_shorttime_counter, &nex_but_stepback, &nex_but_stepnxt,
    &nex_but_reset_cycle, &nex_but_slider1_left, &nex_but_slider1_right,
    &nex_switch_play_pause, &nex_switch_mode, &nex_zyl_messer,
    &nex_zyl_klemmblock, &nex_zyl_feder_zuluft, &nex_zyl_feder_abluft,
    &nex_zyl_wippenhebel, &nex_mot_band_unten, &nex_zyl_schweisstaste,
    &nex_einschaltventil,
    // PAGE 3:
    &nexButton1Left, &nexButton1Right, &nexButton2Left, &nexButton2Right,
    &nexButton3Left, &nexButton3Right,
    // END OF DECLARATION
    NULL // String terminated
};
//***************************************************************************
// END OF TOUCH EVENT LIST
//***************************************************************************
//***************************************************************************
// TOUCH EVENT FUNCTIONS //PushCallback = Press event //PopCallback = Release
// event
//***************************************************************************
//*************************************************
// TOUCH EVENT FUNCTIONS PAGE 1 - LEFT SIDE
//*************************************************
void nex_switch_play_pausePushCallback(void *ptr) {
  machine_running = !machine_running;
  if (machine_running) {
    clearance_next_step = true;
  }
  nex_state_machine_running = !nex_state_machine_running;
}
void nex_switch_modePushCallback(void *ptr) {
  state_controller.toggle_step_auto_mode();
  step_mode = state_controller.is_in_step_mode();
  Serial2.print("click bt1,1"); // CLICK BUTTON
  send_to_nextion();
}
void nex_but_stepbackPushCallback(void *ptr) {
  state_controller.set_machine_stop();
  reset_flag_of_current_step();
  state_controller.set_step_mode();
  state_controller.switch_to_previous_step();
  reset_flag_of_current_step();
}
void nex_but_stepnxtPushCallback(void *ptr) {
  state_controller.set_machine_stop();
  reset_flag_of_current_step();
  state_controller.set_step_mode();
  state_controller.switch_to_next_step();
  reset_flag_of_current_step();
}
void nex_but_reset_cyclePushCallback(void *ptr) {
  cycle_step = 0;
  step_mode = true;
}
//*************************************************
// TOUCH EVENT FUNCTIONS PAGE 1 - RIGHT SIDE
//*************************************************
void nex_zyl_feder_zuluftPushCallback(void *ptr) {
  zyl_feder_zuluft.toggle();
  nex_state_zyl_feder_zuluft = !nex_state_zyl_feder_zuluft;
}

void nex_zyl_feder_abluftPushCallback(void *ptr) {
  zyl_feder_abluft.toggle();
  nex_state_zyl_feder_abluft = !nex_state_zyl_feder_abluft;
}

void nex_zyl_klemmblockPushCallback(void *ptr) {
  zyl_klemmblock.toggle();
  nex_state_zyl_klemmblock = !nex_state_zyl_klemmblock;
}

void nex_zyl_wippenhebelPushCallback(void *ptr) { zyl_wippenhebel.set(1); }

void nex_zyl_wippenhebelPopCallback(void *ptr) { zyl_wippenhebel.set(0); }

void nex_mot_band_untenPushCallback(void *ptr) { zyl_spanntaste.set(1); }
void nex_mot_band_untenPopCallback(void *ptr) { zyl_spanntaste.set(0); }
void nex_zyl_schweisstastePushCallback(void *ptr) { zyl_schweisstaste.set(1); }
void nex_zyl_schweisstastePopCallback(void *ptr) { zyl_schweisstaste.set(0); }
void nex_zyl_messerPushCallback(void *ptr) { zyl_messer.set(1); }
void nex_zyl_messerPopCallback(void *ptr) { zyl_messer.set(0); }
void nex_einschaltventilPushCallback(void *ptr) {
  einschaltventil.toggle();
  nex_state_einschaltventil = !nex_state_einschaltventil;
}

//*************************************************
// TOUCH EVENT FUNCTIONS PAGE 2 - LEFT SIDE
//*************************************************

void nex_but_slider1_leftPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(startfuelldauer) - 100;
  eepromCounter.set_value(startfuelldauer, newValue);
  if (eepromCounter.get_value(startfuelldauer) < 0) {
    eepromCounter.set_value(startfuelldauer, 0);
  }
}

void nex_but_slider1_rightPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(startfuelldauer) + 100;
  eepromCounter.set_value(startfuelldauer, newValue);
  if (eepromCounter.get_value(startfuelldauer) > 7000) {
    eepromCounter.set_value(startfuelldauer, 7000);
  }
}

//*************************************************
// TOUCH EVENT FUNCTIONS PAGE 2 - RIGHT SIDE
//*************************************************
void nex_but_reset_shorttime_counterPushCallback(void *ptr) {
  eepromCounter.set_value(shorttimeCounter, 0);
  // RESET LONGTIME COUNTER IF RESET BUTTON IS PRESSED LONG ENOUGH:
  counter_reset_stopwatch = millis();
  reset_stopwatch_active = true;
}

void nex_but_reset_shorttime_counterPopCallback(void *ptr) {
  reset_stopwatch_active = false;
}
//*************************************************
// TOUCH EVENT FUNCTIONS PAGE 3
//*************************************************
void nexButton1LeftPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(cyclesInARow) - 1;
  eepromCounter.set_value(cyclesInARow, newValue);
  if (eepromCounter.get_value(cyclesInARow) < 0) {
    eepromCounter.set_value(cyclesInARow, 0);
  }
}

void nexButton1RightPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(cyclesInARow) + 1;
  eepromCounter.set_value(cyclesInARow, newValue);
  if (eepromCounter.get_value(cyclesInARow) > 10) {
    eepromCounter.set_value(cyclesInARow, 10);
  }
}

void nexButton2LeftPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(longCooldownTime) - 10;
  eepromCounter.set_value(longCooldownTime, newValue);
  if (eepromCounter.get_value(longCooldownTime) < 0) {
    eepromCounter.set_value(longCooldownTime, 0);
  }
}

void nexButton2RightPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(longCooldownTime) + 10;
  eepromCounter.set_value(longCooldownTime, newValue);
  if (eepromCounter.get_value(longCooldownTime) > 600) {
    eepromCounter.set_value(longCooldownTime, 600);
  }
}

void nexButton3LeftPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(strapEjectFeedTime) - 1;
  eepromCounter.set_value(strapEjectFeedTime, newValue);
  if (eepromCounter.get_value(strapEjectFeedTime) < 0) {
    eepromCounter.set_value(strapEjectFeedTime, 0);
  }
}
void nexButton3RightPushCallback(void *ptr) {
  long newValue = eepromCounter.get_value(strapEjectFeedTime) + 1;
  eepromCounter.set_value(strapEjectFeedTime, newValue);
  if (eepromCounter.get_value(strapEjectFeedTime) > 20) {
    eepromCounter.set_value(strapEjectFeedTime, 20);
  }
}

//*************************************************
// TOUCH EVENT FUNCTIONS PAGE CHANGES
//*************************************************
void nex_page0PushCallback(void *ptr) { nex_current_page = 0; }

void nex_page1PushCallback(void *ptr) {
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

void nex_page2PushCallback(void *ptr) {
  nex_current_page = 2;
  // REFRESH BUTTON STATES:
  nex_prev_startfuelldauer = 0;
  nex_prev_shorttime_counter = 0;
  nex_prev_longtime_counter = 0;
  nex_prev_federkraft_int = 0;
  nex_prev_federdruck = 10000;
}

void nex_page3PushCallback(void *ptr) {
  nex_current_page = 3;
  // REFRESH BUTTON STATES:
  nexPrevCyclesInARow = 0;
  nexPrevLongCooldownTime = 0;
  nexPrevStrapEjectFeedTime = 0;
}
//***************************************************************************
// END OF TOUCH EVENT FUNCTIONS
//***************************************************************************

//***************************************************************************

//***************************************************************************
void nextion_setup()
//***************************************************************************
//***************************************************************************
{                      // START NEXTION SETUP
  Serial2.begin(9600); // Start serial comunication at baud=9600

  //***************************************************************************
  // INCREASE BAUD RATE
  //***************************************************************************
  /*
   delay(500);
   Serial2.print("baud=38400");
   send_to_nextion();
   Serial2.end();
   Serial2.begin(38400);
   */
  //***************************************************************************
  // REGISTER THE EVENT CALLBACK FUNCTIONS
  //***************************************************************************
  // PAGE 0 1 2:
  nex_page0.attachPush(nex_page0PushCallback);
  nex_page1.attachPush(nex_page1PushCallback);
  nex_page2.attachPush(nex_page2PushCallback);
  nex_page3.attachPush(nex_page3PushCallback);
  nex_but_stepback.attachPush(nex_but_stepbackPushCallback);
  nex_but_stepnxt.attachPush(nex_but_stepnxtPushCallback);
  nex_zyl_klemmblock.attachPush(nex_zyl_klemmblockPushCallback);
  nex_but_reset_cycle.attachPush(nex_but_reset_cyclePushCallback);
  nex_but_slider1_left.attachPush(nex_but_slider1_leftPushCallback);
  nex_but_slider1_right.attachPush(nex_but_slider1_rightPushCallback);
  nex_but_stepback.attachPush(nex_but_stepbackPushCallback);
  nex_but_stepnxt.attachPush(nex_but_stepnxtPushCallback);
  nex_switch_mode.attachPush(nex_switch_modePushCallback);
  nex_switch_play_pause.attachPush(nex_switch_play_pausePushCallback);
  nex_zyl_klemmblock.attachPush(nex_zyl_klemmblockPushCallback);
  nex_zyl_feder_zuluft.attachPush(nex_zyl_feder_zuluftPushCallback);
  nex_zyl_feder_abluft.attachPush(nex_zyl_feder_abluftPushCallback);
  nex_einschaltventil.attachPush(nex_einschaltventilPushCallback);
  // PAGE 3:
  nexButton1Left.attachPush(nexButton1LeftPushCallback);
  nexButton1Right.attachPush(nexButton1RightPushCallback);
  nexButton2Left.attachPush(nexButton2LeftPushCallback);
  nexButton2Right.attachPush(nexButton2RightPushCallback);
  nexButton3Left.attachPush(nexButton3LeftPushCallback);
  nexButton3Right.attachPush(nexButton3RightPushCallback);

  //*****PUSH+POP:
  nex_zyl_wippenhebel.attachPush(nex_zyl_wippenhebelPushCallback);
  nex_zyl_wippenhebel.attachPop(nex_zyl_wippenhebelPopCallback);
  nex_mot_band_unten.attachPush(nex_mot_band_untenPushCallback);
  nex_mot_band_unten.attachPop(nex_mot_band_untenPopCallback);
  nex_zyl_schweisstaste.attachPush(nex_zyl_schweisstastePushCallback);
  nex_zyl_schweisstaste.attachPop(nex_zyl_schweisstastePopCallback);
  nex_zyl_messer.attachPush(nex_zyl_messerPushCallback);
  nex_zyl_messer.attachPop(nex_zyl_messerPopCallback);
  nex_but_reset_shorttime_counter.attachPush(
      nex_but_reset_shorttime_counterPushCallback);
  nex_but_reset_shorttime_counter.attachPop(
      nex_but_reset_shorttime_counterPopCallback);

  //***************************************************************************
  // END OF REGISTER
  //***************************************************************************
  delay(3000);
  sendCommand("page 1"); // SWITCH NEXTION TO PAGE X
  send_to_nextion();

} // END OF NEXTION SETUP

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
  if (state_controller.is_in_step_mode() ||
      state_controller.is_in_auto_mode()) {
    update_main_cycle_name();
  }
}

String get_main_cycle_display_string() {
  int current_step = state_controller.get_current_step();
  String display_text_cycle_name =
      main_cycle_steps[current_step]->get_display_text();
  return display_text_cycle_name;
}
// NEXTION GENERAL DISPLAY FUNCTIONS *******************************************

void send_to_nextion() {
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
}

// void update_display_counter() {
//   long new_value = counter.get_value(longtime_counter);
//   Serial2.print("t0.txt=");
//   Serial2.print("\"");
//   Serial2.print(new_value);
//   Serial2.print("\"");
//   send_to_nextion();
// }

void show_info_field() {
  if (nex_current_page == 1) {
    Serial2.print("vis t4,1");
    send_to_nextion();
  }
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

void clear_text_field(String textField) {
  Serial2.print(textField);
  Serial2.print(".txt=");
  Serial2.print("\"");
  Serial2.print(""); // erase text
  Serial2.print("\"");
  send_to_nextion();
}

void display_value_in_field(int value, String valueField) {
  Serial2.print(valueField);
  Serial2.print(".val=");
  Serial2.print(value);
  send_to_nextion();
}

void display_text_in_field(String text, String textField) {
  Serial2.print(textField);
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
//***************************************************************************
void nextion_loop()
//***************************************************************************
{ // START NEXTION LOOP

  nexLoop(nex_listen_list); // check for any touch event
  //***************************************************************************
  if (nex_current_page == 1) // START PAGE 1
  {
    //*******************
    // PAGE 1 - LEFT SIDE:
    //*******************
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

    //*******************
    // PAGE 1 - RIGHT SIDE:
    //*******************
    // UPDATE CYCLE NAME

    update_cycle_name();
    // if (nex_prev_cycle_step != cycle_step) {
    //   Serial2.print("t0.txt=");
    //   Serial2.print("\"");
    //   Serial2.print(cycle_step + 1); // write the number of the step
    //   Serial2.print(" ");
    //   Serial2.print(cycle_name[cycle_step]); // write the name of the step
    //   Serial2.print("\"");
    //   send_to_nextion();
    //   nex_prev_cycle_step = cycle_step;
    // }
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
  //*******************
  // PAGE 2 - LEFT SIDE
  //*******************
  if (nex_current_page == 2) // START PAGE 2
  {
    if (nex_prev_startfuelldauer != eepromCounter.get_value(startfuelldauer)) {
      send_to_nextion();
      Serial2.print("t4.txt=");
      Serial2.print("\"");
      Serial2.print(eepromCounter.get_value(startfuelldauer));
      Serial2.print(" ms");
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_startfuelldauer = eepromCounter.get_value(startfuelldauer);
    }

    if (nex_prev_federkraft_int != federkraft_int &&
        millis() > nex_update_timer) {
      // update force:
      Serial2.print("h1.val=");
      Serial2.print(federkraft_int);
      send_to_nextion();

      Serial2.print("t6.txt=");
      Serial2.print("\"");
      Serial2.print(federkraft_int);
      Serial2.print(" N");
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_federkraft_int = federkraft_int;

      // update pressure:
      Serial2.print("t8.txt=");
      Serial2.print("\"");
      Serial2.print(federdruck_float, 1);
      Serial2.print(" bar");
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_federdruck = federdruck_float;

      nex_update_timer = millis() + 200;
    }

    //*******************
    // PAGE 2 - RIGHT SIDE
    //*******************
    if (nex_prev_longtime_counter != eepromCounter.get_value(longtimeCounter)) {
      Serial2.print("t10.txt=");
      Serial2.print("\"");
      Serial2.print(eepromCounter.get_value(longtimeCounter));
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_longtime_counter = eepromCounter.get_value(longtimeCounter);
    }
    if (nex_prev_shorttime_counter !=
        eepromCounter.get_value(shorttimeCounter)) {
      Serial2.print("t12.txt=");
      Serial2.print("\"");
      Serial2.print(eepromCounter.get_value(shorttimeCounter));
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_shorttime_counter = eepromCounter.get_value(shorttimeCounter);
    }
    if (reset_stopwatch_active) {
      if (millis() - counter_reset_stopwatch > 5000) {
        eepromCounter.set_value(shorttimeCounter, 0);
        eepromCounter.set_value(longtimeCounter, 0);
      }
    }
  }
  //*******************
  // PAGE 3
  //*******************
  if (nex_current_page == 3) { // START PAGE 3
    if (nexPrevCyclesInARow != eepromCounter.get_value(cyclesInARow)) {
      send_to_nextion();
      Serial2.print("t4.txt=");
      Serial2.print("\"");
      Serial2.print(eepromCounter.get_value(cyclesInARow));
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_startfuelldauer = eepromCounter.get_value(cyclesInARow);
    }

    if (nexPrevLongCooldownTime != eepromCounter.get_value(longCooldownTime)) {
      Serial2.print("t5.txt=");
      Serial2.print("\"");
      // Serial2.print("222");
      Serial2.print(eepromCounter.get_value(longCooldownTime));
      Serial2.print(" s");
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_longtime_counter = eepromCounter.get_value(longCooldownTime);
    }

    if (nexPrevStrapEjectFeedTime !=
        eepromCounter.get_value(strapEjectFeedTime)) {
      Serial2.print("t7.txt=");
      Serial2.print("\"");
      Serial2.print(eepromCounter.get_value(strapEjectFeedTime));
      Serial2.print(" s");
      Serial2.print("\"");
      send_to_nextion();
      nex_prev_longtime_counter = eepromCounter.get_value(strapEjectFeedTime);
    }
  }

} // END OF NEXTION LOOP

void read_n_toggle() {

  // IN AUTO MODE, MACHINE RUNS FROM STEP TO STEP AUTOMATICALLY:
  if (!step_mode) //=AUTO MODE
  {
    clearance_next_step = true;
  }

  // IN STEP MODE, MACHINE STOPS AFTER EVERY COMPLETED CYCLYE:
  if (step_mode && !clearance_next_step) {
    machine_running = false;
  }

  // START TEST RIG:
  if (digitalRead(start_button)) {
    machine_running = true;
    clearance_next_step = true;
  }

  // STOP TEST_RIG:
  if (digitalRead(stop_button)) {
    machine_running = false;
  }

  // BANDSENSOREN ABFRAGEN:

  if (digitalRead(bandsensor_oben) && digitalRead(bandsensor_unten)) {
    band_vorhanden = true;
    error_blink = false;
  } else {
    band_vorhanden = false;
    error_blink = true;
    clearance_next_step = false;
    machine_running = false;
  }

  // START- UND ENDPOSITIONSSCHALTER ABFRAGEN:
  startposition_erreicht = digitalRead(taster_startposition);
  endposition_erreicht = digitalRead(taster_endposition);

  //*****************************************************************************
  // READ PRESSURE SENSOR AND CALCULATE PRESSURE AND AIR-SPRING-FORCE:
  // DRUCKSENSOR 0-10V => 0-12bar
  // CONTROLLINO ANALOG INPUT VALUE 0-1023, 30mV per digit (controlino.biz)
  // 10V   => analogRead 333.3 (10V/30mV)
  // 12bar => anlaogRead 333.3
  // 1bar  => analogRead 27.778

  federdruck_float = analogRead(drucksensor) / 27.778; //[bar]

  // SMOOTH VALUES:
  federdruck_smoothed = ((federdruck_smoothed * 4 + federdruck_float) / 5);
  // CONVERT TO INT AND [mbar]:
  federdruck_mbar_int = federdruck_smoothed * 1000; //[mbar]

  // CALM FUNCTION:
  // To prevent flickering, the pressure value will only be updated
  // if there's a higher or lower value five times in a row

  if (federdruck_mbar_int > federdruck_beruhigt) {
    if (calmcounter >= 0) {
      calmcounter++;
      calmcountersum = calmcountersum + federdruck_mbar_int;
    }
    if (calmcounter < 0) // RESET CALMCOUNTER
    {
      calmcounter = 0;
      calmcountersum = 0;
    }
  }
  if (federdruck_mbar_int < federdruck_beruhigt) {
    if (calmcounter <= 0) {
      calmcounter--;
      calmcountersum = calmcountersum + federdruck_mbar_int;
    }
    if (calmcounter > 0) // RESET CALMCOUNTER
    {
      calmcounter = 0;
      calmcountersum = 0;
    }
  }
  if (abs(calmcounter) == 5) // abs()=> Absolutwert
  {
    // update value only if there is a significant difference to the previous
    // value:
    if (abs(calmcountersum / abs(calmcounter) - prev_federdruck_beruhigt) >
        50) {
      federdruck_beruhigt = calmcountersum / abs(calmcounter);
      prev_federdruck_beruhigt = federdruck_beruhigt;
    }
    calmcounter = 0;
    calmcountersum = 0;
  }

  // GO BACK TO FLOAT TO GET A "X.X bar" PRINT ON THE DISPLAY:

  federdruck_float = federdruck_beruhigt;
  federdruck_float = federdruck_float / 1000; //[bar]

  // CALCULATE FORCE:
  federkraft_int =
      federdruck_float *
      1472.6; // 1bar  => 1472.6N (Dauertest BXT 3-32 Zylinderkraft.xlsx)

  // SET LAST DIGIT ZERO
  federkraft_int = federkraft_int / 10;
  federkraft_int = federkraft_int * 10;

} // END OF READ_N_TOGGLE

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

  void do_initial_stuff() {
    feed_time = eepromCounter.get_value(strapEjectFeedTime) * 1000;
  };
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
    startfuelltimer = millis();
    startfuellung_running = true;
    zyl_feder_zuluft.set(1); // 1=füllen 0=geschlossen
    zyl_feder_abluft.set(1); // 1=geschlossen 0=entlüften
  };

  void do_loop_stuff() {
    if ((millis() - startfuelltimer) >
        eepromCounter.get_value(startfuelldauer)) {
      zyl_feder_zuluft.set(0); // 1=füllen 0=geschlossen
      startfuellung_running = false;
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
    if (federdruck_float < 0.1) // warten bis der Druck ist abgebaut
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
      zyl_feder_zuluft.set(0);    // 1=füllen 0=geschlossen
      zyl_feder_abluft.set(0);    // 1=geschlossen 0=entlüften
      if (federdruck_float < 0.1) // warten bis der Druck abgebaut ist
      {
        if (cycle_step_delay.delay_time_is_up(500)) {
          eepromCounter.count_one_up(shorttimeCounter);
          eepromCounter.count_one_up(longtimeCounter);
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
    abkuehldauer = eepromCounter.get_value(longCooldownTime) * 1000;
  };

  void do_loop_stuff() {
    if (testZyklenZaehler == eepromCounter.get_value(cyclesInARow)) {
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
  eepromCounter.setup(eepromMinAddress, eepromMaxAddress, numberOfEepromValues);
  Serial.begin(115200); // start serial connection

  nextion_setup();

  pinMode(stop_button, INPUT);
  pinMode(start_button, INPUT);
  pinMode(bandsensor_oben, INPUT);
  pinMode(bandsensor_unten, INPUT);
  pinMode(taster_startposition, INPUT);
  pinMode(taster_endposition, INPUT);
  pinMode(drucksensor, INPUT);

  pinMode(green_light, OUTPUT);
  pinMode(red_light, OUTPUT);
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
    if (state_controller.is_in_auto_mode() &&
        state_controller.machine_is_running()) {
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
  read_n_toggle();
  nextion_loop();

  // UPDATE DISPLAY:

  // MONITOR EMERGENCY SIGNAL:
  // monitor_emergency_signal();

  // DO NOT WATCH TIMEOUTS IF MACHINE IS NOT RUNNING (PAUSE):
  if (!state_controller.machine_is_running()) {
    // machine_stopped_error_timeout.reset_time();
    // bandsensor_timeout.reset_time();
  }

  // MONITOR ERRORS ONLY WHEN RIG IS RUNNING IN AUTO MODE:
  if (state_controller.machine_is_running() &&
      state_controller.is_in_auto_mode()) {
    // monitor_error_timeouts();
    // monitor_temperature_error();
  }

  // RUN STEP OR AUTO MODE:
  if (state_controller.is_in_step_mode() ||
      state_controller.is_in_auto_mode()) {
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
