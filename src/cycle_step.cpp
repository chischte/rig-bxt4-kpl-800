#include "cycle_step.h"
#include <ArduinoSTL.h>

Cycle_step::Cycle_step() { //
  object_count++;
}

void Cycle_step::do_stuff() {
  if (!_innit_completed) {
    do_initial_stuff();
    _innit_completed = true;
  } else {
    do_loop_stuff();
  }
}

void Cycle_step::reset_flags() {
  _innit_completed = false;
  _loop_completed = false;
}

void Cycle_step::set_loop_completed() { //
  _loop_completed = true;
}

// This is a "one time flag", state will be reseted after fist inquiry:
bool Cycle_step::is_completed() {
  if (_loop_completed) {
    _loop_completed = false;
    _innit_completed = false;
    return true;
  } else {
    return false;
  }
}
