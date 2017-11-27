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
#define CMD_PLAN 7
#define CMD_ENABLE_PID 8
#define CMD_XYZ_VALUE 9
#define CMD_ANGLES 10
#define CMD_SET_PID 11
#define CMD_LOOP_TIME 12
#define CMD_LEG_NUMBER 13
#define CMD_PWM_LIMITS 14
#define CMD_ADC_LIMITS 15
#define CMD_CALF_SCALE 16

void on_estop(CommandProtocol *cmd){
  byte severity = ESTOP_DEFAULT;
  if (cmd->has_arg()) {
    severity = cmd->get_arg<byte>();
  }
  leg->estop->set_estop(severity);
};

void on_heartbeat(CommandProtocol *cmd) {
  leg->estop->set_heartbeat();
  // send heartbeat back
  cmd->start_command(CMD_HEARTBEAT);
  cmd->finish_command();
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
  leg->knee_valve->set_ratio(knee_pwm);
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

void on_plan(CommandProtocol *cmd) {
  // mode, frame, linear(xyz), angular(xyz), speed, start_time
  if (!cmd->has_arg()) return;
  PlanStruct new_plan;
  new_plan.mode = cmd->get_arg<byte>();
  if (!cmd->has_arg()) return;
  new_plan.frame = cmd->get_arg<byte>();
  if (!cmd->has_arg()) return;
  new_plan.linear.x = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  new_plan.linear.y = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  new_plan.linear.z = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  new_plan.angular.x = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  new_plan.angular.y = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  new_plan.angular.z = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  new_plan.speed = cmd->get_arg<float>();
  if (cmd->has_arg()) {
    new_plan.start_time = cmd->get_arg<float>();
  } else {
    new_plan.start_time = millis();
  }
  leg->set_next_plan(new_plan);
}

void on_enable_pid(CommandProtocol *cmd) {
  bool e = true;
  if (cmd->has_arg()) {
    e = cmd->get_arg<bool>();
  }
  if (e) {
    leg->enable_pids();
  } else {
    leg->disable_pids();
  };
}

void on_set_pid(CommandProtocol *cmd) {
    // joint [0:hip, 1:thigh, 2:knee]
    // p,i,d,min_value,max_value
    if (!cmd->has_arg()) return;
    byte ji = cmd->get_arg<byte>();
    if (ji > 2) return;
    if (!cmd->has_arg()) return;
    float p = cmd->get_arg<float>();
    if (!cmd->has_arg()) return;
    float i = cmd->get_arg<float>();
    if (!cmd->has_arg()) return;
    float d = cmd->get_arg<float>();
    if (!cmd->has_arg()) return;
    float mino = cmd->get_arg<float>();
    if (!cmd->has_arg()) return;
    float maxo = cmd->get_arg<float>();
    if (maxo <= mino) return;
    PID* pid;
    switch (ji) {
        case 0:
            pid = leg->hip_pid;
            break;
        case 1:
            pid = leg->thigh_pid;
            break;
        case 2:
            pid = leg->knee_pid;
            break;
    }
    // disable pid (estop?)
    leg->disable_pids();
    // set values
    pid->set_p(p); pid->set_i(i); pid->set_d(d);
    pid->set_output_limits(mino, maxo);
    pid->reset();
    // re-enable pid? need to do this outside the loop
};

void on_leg_number(CommandProtocol *cmd) {
  // write leg number
  if (cmd->has_arg()) {
    byte v = cmd->get_arg<byte>();
    if (v <= LEGNUMBER_MAX) {
      // write to eeprom
      write_leg_number_to_eeprom((LEG_NUMBER)(v));
      leg->set_leg_number((LEG_NUMBER)(v));
    };
  };
  // return leg number
  cmd->start_command(CMD_LEG_NUMBER);
  cmd->add_arg((byte)(leg->leg_number));
  cmd->finish_command();
}

void on_pwm_limits(CommandProtocol *cmd) {
  // joint index, extend min, extend max, retract min, retract max
  if (!cmd->has_arg()) return;
  byte ji = cmd->get_arg<byte>();
  if (ji > 2) return;
  if (!cmd->has_arg()) return;
  float emin = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float emax = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float rmin = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float rmax = cmd->get_arg<float>();
  if (emax <= emin) return;
  if (rmax <= rmin) return;
  Valve* valve;
  switch (ji) {
      case 0:
          valve = leg->hip_valve;
          break;
      case 1:
          valve = leg->thigh_valve;
          break;
      case 2:
          valve = leg->knee_valve;
          break;
  }
  // disable pid (estop?)
  leg->disable_pids();
  valve->set_pwm_limits(emin, emax, rmin, rmax);
  //pid->reset();
  // re-enable pid? need to do this outside the loop
}

void on_adc_limits(CommandProtocol *cmd) {
  // joint index, adc_min, adc_max
  if (!cmd->has_arg()) return;
  byte ji = cmd->get_arg<byte>();
  if (ji > 2) return;
  if (!cmd->has_arg()) return;
  float amin = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float amax = cmd->get_arg<float>();
  if (amax <= amin) return;
  StringPot* pot;
  switch (ji) {
      case 0:
          pot = leg->hip_pot;
          break;
      case 1:
          pot = leg->thigh_pot;
          break;
      case 2:
          pot = leg->knee_pot;
          break;
  }
  // disable pid (estop?)
  leg->disable_pids();
  pot->set_adc_range(amin, amax);
  //pid->reset();
  // re-enable pid? need to do this outside the loop
}

void on_calf_scale(CommandProtocol *cmd) {
  if (!cmd->has_arg()) return;
  float s = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float o = cmd->get_arg<float>();
  leg->calf_load_transform->set_slope(s);
  leg->calf_load_transform->set_offset(o);
}

void setup(){
  Serial.begin(9600);
  //leg->set_leg_number(LEG_NUMBER::FR);

  com.register_protocol(0, cmd);
  cmd.register_callback(CMD_ESTOP, on_estop);
  cmd.register_callback(CMD_HEARTBEAT, on_heartbeat);
  cmd.register_callback(CMD_PWM, on_pwm);
  cmd.register_callback(CMD_ADC_TARGET, on_adc_target);
  cmd.register_callback(CMD_PLAN, on_plan);
  cmd.register_callback(CMD_ENABLE_PID, on_enable_pid);
  cmd.register_callback(CMD_SET_PID, on_set_pid);
  cmd.register_callback(CMD_LEG_NUMBER, on_leg_number);
  cmd.register_callback(CMD_PWM_LIMITS, on_pwm_limits);
  cmd.register_callback(CMD_ADC_LIMITS, on_adc_limits);
  cmd.register_callback(CMD_CALF_SCALE, on_calf_scale);
}

void loop() {
  com.handle_stream();
  //unsigned long t0 = micros();
  leg->update();
  //t0 = micros() - t0;
  if (adc_timer > ADC_REPORT_TIME) {
    /*
    cmd.start_command(CMD_LOOP_TIME);
    cmd.add_arg(t0);
    cmd.finish_command();
    */
    
    cmd.start_command(CMD_ADC_VALUE);
    cmd.add_arg(leg->hip_analog_sensor->get_adc_value());
    cmd.add_arg(leg->thigh_analog_sensor->get_adc_value());
    cmd.add_arg(leg->knee_analog_sensor->get_adc_value());
    cmd.add_arg(leg->calf_analog_sensor->get_adc_value());
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
    cmd.add_arg(leg->hip_pid->get_setpoint());
    cmd.add_arg(leg->thigh_pid->get_setpoint());
    cmd.add_arg(leg->knee_pid->get_setpoint());

    cmd.add_arg(leg->hip_pid->get_error());
    cmd.add_arg(leg->thigh_pid->get_error());
    cmd.add_arg(leg->knee_pid->get_error());

    cmd.finish_command();

    cmd.start_command(CMD_PWM_VALUE);
    cmd.add_arg(leg->hip_valve->get_pwm());
    cmd.add_arg(leg->thigh_valve->get_pwm());
    cmd.add_arg(leg->knee_valve->get_pwm());
    cmd.finish_command();

    leg->compute_foot_position();
    // angles
    cmd.start_command(CMD_ANGLES);
    cmd.add_arg(leg->joint_angles.hip);
    cmd.add_arg(leg->joint_angles.thigh);
    cmd.add_arg(leg->joint_angles.knee);
    cmd.add_arg(leg->calf_load);
    cmd.add_arg(leg->foot_position_valid);
    cmd.finish_command();

    cmd.start_command(CMD_XYZ_VALUE);
    cmd.add_arg(leg->foot_position.x);
    cmd.add_arg(leg->foot_position.y);
    cmd.add_arg(leg->foot_position.z);
    cmd.finish_command();

    adc_timer = 0;
  }
}

