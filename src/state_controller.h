/* *****************************************************************************
 * state_controller.h **********************************************************
 * *****************************************************************************
 * MAIN OPERATION MODES :
 * 
 * 1) step_mode
 * 
 * 2) auto_mode
 * 
 * 3) error_mode (error mode can run independently, on top of other modes)
 *
 * 4) reset mode (reset can run independently, on top of other modes)
 * 
 * *****************************************************************************
 */

#ifndef StateController_H_
#define StateController_H_

class State_controller {

public:
  // FUNCTIONS:
  State_controller(int number_of_steps);
  State_controller();

  void set_no_of_steps(int number_of_steps);

  void set_step_mode();
  bool is_in_step_mode();

  void set_auto_mode();
  bool is_in_auto_mode();

  void set_error_mode(bool error_mode);
  bool is_in_error_mode();

  void set_reset_mode(bool reset_state);
  bool reset_mode_is_active();

  void set_run_after_reset(bool run_after_reset);
  bool run_after_reset_is_active();

  void set_machine_running(bool machine_state);
  void set_machine_running();
  void set_machine_stop();
  void toggle_machine_running_state();
  void toggle_step_auto_mode();
  bool machine_is_running();

  void switch_to_next_step();
  void switch_to_previous_step();
  void set_current_step_to(int cycle_step);
  int get_current_step();
  bool step_switch_has_happend();


  // VARIABLES:
  // n.a.

private:
  // FUNCTIONS:
  // n.a.

  // VARIABLES:
  int _number_of_main_cycle_steps;
  int _current_main_cycle_step;
  int _previous_cycle_step;
  bool _machine_running;
  bool _step_mode;
  bool _auto_mode;
  bool _reset_mode;
  bool _run_after_reset;
  bool _error_mode;
};
#endif /* StateController_H_ */
