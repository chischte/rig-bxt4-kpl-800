/*******************************************************************************
 * state_controller.cpp ********************************************************
 *******************************************************************************/

#include "state_controller.h"

// CONSTRUCTORS ----------------------------------------------------------------
State_controller::State_controller(int number_of_steps) { _number_of_main_cycle_steps = number_of_steps; }
State_controller::State_controller() {}

void State_controller::set_no_of_steps(int number_of_steps) { _number_of_main_cycle_steps = number_of_steps; }

// STEP MODE -------------------------------------------------------------------
void State_controller::set_step_mode() {
  _step_mode = true;
  _auto_mode = false;
}
bool State_controller::is_in_step_mode() { return _step_mode; }

// AUTO MODE -------------------------------------------------------------------
void State_controller::set_auto_mode() {
  _step_mode = false;
  _auto_mode = true;
}
bool State_controller::is_in_auto_mode() { return _auto_mode; }

// ERROR MODE -------------------------------------------------------------
void State_controller::set_error_mode(bool error_mode) {
  _error_mode = error_mode;
}

bool State_controller::is_in_error_mode() { return _error_mode; }

// RESET MODE ------------------------------------------------------------------
void State_controller::set_reset_mode(bool reset_mode) { _reset_mode = reset_mode; }

bool State_controller::reset_mode_is_active() {
  bool reset_mode = _reset_mode;
  return reset_mode;
}

void State_controller::set_run_after_reset(bool run_after_reset) { _run_after_reset = run_after_reset; }

bool State_controller::run_after_reset_is_active() {
  bool run_after_reset = _run_after_reset;
  return run_after_reset;
}

// MACHINE RUNNING -------------------------------------------------------------

void State_controller::set_machine_running(bool machine_state) { _machine_running = machine_state; }

void State_controller::set_machine_running() { _machine_running = true; }

void State_controller::set_machine_stop() { _machine_running = false; }

void State_controller::toggle_machine_running_state() { _machine_running = !_machine_running; }

void State_controller::toggle_step_auto_mode() {
  if (is_in_auto_mode()) {
    set_step_mode();
  } else {
    set_auto_mode();
  }
}

bool State_controller::machine_is_running() {
  bool machineRunning = _machine_running;
  return machineRunning;
}

// STEP MANAGEMENT FOR ALL MODES ---------------------------------------------
void State_controller::switch_to_next_step() {
  _current_main_cycle_step++;
  if (_current_main_cycle_step == _number_of_main_cycle_steps) {
    _current_main_cycle_step = 0;
  }
}

void State_controller::switch_to_previous_step() {
  if (_current_main_cycle_step > 0) {
    _current_main_cycle_step--;
  }
}

int State_controller::get_current_step() {
  int current_cycle_step;
  current_cycle_step = _current_main_cycle_step;
  return current_cycle_step;
}

bool State_controller::step_switch_has_happend() {
  bool step_has_changed = (_previous_cycle_step != get_current_step());
  _previous_cycle_step = get_current_step();
  return step_has_changed;
}

void State_controller::set_current_step_to(int cycle_step) { _current_main_cycle_step = cycle_step; }
