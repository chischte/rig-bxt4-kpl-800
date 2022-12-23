#ifndef CYCLESTEP_H
#define CYCLESTEP_H
#include <ArduinoSTL.h>

class Cycle_step {
public:
  // VARIABLES:
  static int object_count;

  // FUNCTIONS:
  Cycle_step();
  void do_stuff();
  void reset_flags();

  // VIRTUAL FUNCTIONS:
  virtual void do_initial_stuff() = 0;
  virtual void do_loop_stuff() = 0;

  // SETTER:
  void set_loop_completed();

  // GETTER:
  bool is_completed();
  virtual String get_display_text() = 0;

private:
  // VARIABLES:
  bool _loop_completed;
  bool _innit_completed;

  // FUNCTIONS:
  //n.A.
};

#endif
