#include "leg.h"
#include <comando.h>

Comando com = Comando(Serial);
CommandProtocol cmd = CommandProtocol(com);

Leg* leg = new Leg();

#define ADC_REPORT_TIME 20
elapsedMillis adc_timer;

#define CMD_ESTOP 0
#define CMD_HEARTBEAT 1
#define CMD_PWM 2
#define CMD_ADC_VALUE 3
#define CMD_ADC_TARGET 4
#define CMD_PWM_VALUE 5
#define CMD_PID_OUTPUT 6

void on_estop(CommandProtocol *cmd){
  byte severity = ESTOP_DEFAULT;
  if (cmd->has_arg()) {
    severity = cmd->get_arg<byte>();
  }
  leg->estop->set_estop(severity);
};

void on_heartbeat(CommandProtocol *cmd) {
  leg->estop->set_heartbeat();
};

void on_pwm(CommandProtocol *cmd) {
  float hip_pwm = 0;
  float thigh_pwm = 0;
  float knee_pwm = 0;
  if (cmd->has_arg()) {
    hip_pwm = cmd->get_arg<float>();
  } else {
    return;
  };
  if (cmd->has_arg()) {
    thigh_pwm = cmd->get_arg<float>();
  } else {
    return;
  };
  if (cmd->has_arg()) {
    knee_pwm = cmd->get_arg<float>();
  } else {
    return;
  };
  leg->hip_valve->set_ratio(hip_pwm);
  leg->thigh_valve->set_ratio(thigh_pwm);
  leg->knee_valve->set_pwm(knee_pwm);
};


void on_adc_target(CommandProtocol *cmd) {
  unsigned int hip_target = 0;
  unsigned int thigh_target = 0;
  unsigned int knee_target = 0;
  if (cmd->has_arg()) {
    hip_target = cmd->get_arg<unsigned int>();
  } else {
    return;
  };
  if (cmd->has_arg()) {
    thigh_target = cmd->get_arg<unsigned int>();
  } else {
    return;
  };
  if (cmd->has_arg()) {
    knee_target = cmd->get_arg<unsigned int>();
  } else {
    return;
  };
  leg->hip_joint->set_target_adc_value(hip_target);
  leg->thigh_joint->set_target_adc_value(thigh_target);
  leg->knee_joint->set_target_adc_value(knee_target);
}

void setup(){
  Serial.begin(9600);
  leg->set_leg_number(LEG_NUMBER::FR);

  com.register_protocol(0, cmd);
  cmd.register_callback(CMD_ESTOP, on_estop);
  cmd.register_callback(CMD_HEARTBEAT, on_heartbeat);
  cmd.register_callback(CMD_PWM, on_pwm);
  cmd.register_callback(CMD_ADC_TARGET, on_adc_target);
}

void loop() {
  com.handle_stream();
  leg->update();
  if (adc_timer > ADC_REPORT_TIME) {
    cmd.start_command(CMD_ADC_VALUE);
    cmd.add_arg(leg->hip_pot->get_adc_value());
    cmd.add_arg(leg->thigh_pot->get_adc_value());
    cmd.add_arg(leg->knee_pot->get_adc_value());
    cmd.finish_command();
    cmd.start_command(CMD_PID_OUTPUT);
    
    cmd.add_arg(leg->hip_joint->get_pid_output());
    cmd.add_arg(leg->thigh_joint->get_pid_output());
    cmd.add_arg(leg->knee_joint->get_pid_output());
    
    /*
    cmd.add_arg(leg->hip_pid->get_output());
    cmd.add_arg(leg->thigh_pid->get_output());
    cmd.add_arg(leg->knee_pid->get_output());
    */
    /*
    cmd.add_arg(leg->hip_pid->get_error());
    cmd.add_arg(leg->thigh_pid->get_error());
    cmd.add_arg(leg->knee_pid->get_error());
    */
    /*
    cmd.add_arg(leg->hip_pid->get_setpoint());
    cmd.add_arg(leg->thigh_pid->get_setpoint());
    cmd.add_arg(leg->knee_pid->get_setpoint());
    */
    cmd.finish_command();
    cmd.start_command(CMD_PWM_VALUE);
    cmd.add_arg(leg->hip_valve->get_pwm());
    cmd.add_arg(leg->thigh_valve->get_pwm());
    cmd.add_arg(leg->knee_valve->get_pwm());
    cmd.finish_command();
    adc_timer = 0;
  }
}

