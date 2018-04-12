#include "leg.h"
#include <comando.h>

Comando com = Comando(Serial);
CommandProtocol cmd = CommandProtocol(com);

Leg* leg = new Leg();

struct ReportFlags {
  bool adc: 1;
  bool pid: 1;
  bool pwm: 1;
  bool xyz: 1;
  bool angles: 1;
  bool loop_time: 1;
};

unsigned long report_time = 20;
elapsedMillis report_timer;
ReportFlags to_report = {
  .adc = true,
  .pid = true,
  .pwm = true,
  .xyz = true,
  .angles = true,
  .loop_time = false,
};

// commands
#define CMD_HEARTBEAT 0

// attributes
#define CMD_ESTOP 1
#define CMD_PWM 2
#define CMD_PLAN 3
#define CMD_ENABLE_PID 4
#define CMD_PID_CONFIG 5
#define CMD_LEG_NUMBER 6
#define CMD_PWM_LIMITS 7
#define CMD_ADC_LIMITS 8
#define CMD_CALF_SCALE 9
#define CMD_REPORT_TIME 10
#define CMD_PID_SEED_TIME 11
#define CMD_RESET_PIDS 12
#define CMD_DITHER 13

#define CMD_REPORT_ADC 21
#define CMD_REPORT_PID 22
#define CMD_REPORT_PWM 23
#define CMD_REPORT_XYZ 24
#define CMD_REPORT_ANGLES 25
#define CMD_REPORT_LOOP_TIME 26

void on_heartbeat(CommandProtocol *cmd) {
  leg->estop->set_heartbeat();
  // send heartbeat back
  cmd->start_command(CMD_HEARTBEAT);
  cmd->finish_command();
};

void on_estop(CommandProtocol *cmd){
  byte severity = ESTOP_DEFAULT;
  bool get = false;
  if (cmd->has_arg()) {
    severity = cmd->get_arg<byte>();
  } else {
    get = true;
  }
  leg->estop->set_estop(severity);
  if (get) {
    cmd->start_command(CMD_ESTOP);
    cmd->add_arg((byte)(severity));
    cmd->finish_command();
  }
};

void send_estop(byte severity) {
  cmd.start_command(CMD_ESTOP);
  cmd.add_arg((byte)(severity));
  cmd.finish_command();
};

void on_pwm(CommandProtocol *cmd) {
  bool get = false;
  float hip_pwm = 0;
  float thigh_pwm = 0;
  float knee_pwm = 0;
  if (cmd->has_arg()) {
    hip_pwm = cmd->get_arg<float>();
  } else {
    get = true;
  };
  if (cmd->has_arg()) {
    thigh_pwm = cmd->get_arg<float>();
  } else {
    get = true;
  };
  if (cmd->has_arg()) {
    knee_pwm = cmd->get_arg<float>();
  } else {
    get = true;
  };
  if (get) {
    cmd->start_command(CMD_PWM);
    cmd->add_arg(leg->hip_valve->get_ratio());
    cmd->add_arg(leg->thigh_valve->get_ratio());
    cmd->add_arg(leg->knee_valve->get_ratio());
    cmd->finish_command();
  } else {
    leg->hip_valve->set_ratio(hip_pwm);
    leg->thigh_valve->set_ratio(thigh_pwm);
    leg->knee_valve->set_ratio(knee_pwm);
  }
};


void on_plan(CommandProtocol *cmd) {
  // mode, frame, linear(xyz), angular(xyz), speed, start_time
  if (!cmd->has_arg()) {
    // return current plan
    return;
  };
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
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_ENABLE_PID);
    cmd->add_arg((bool)(leg->pids_enabled()));
    cmd->finish_command();
    return;
  }
  if (cmd->get_arg<bool>()) {
    leg->enable_pids();
  } else {
    leg->disable_pids();
  };
}

void on_pid_config(CommandProtocol *cmd) {
    // joint [0:hip, 1:thigh, 2:knee]
    // p,i,d,min_value,max_value
    if (!cmd->has_arg()) return;
    byte ji = cmd->get_arg<byte>();
    if (ji > 2) return;
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
        default:
            return;
    }
    if (!cmd->has_arg()) {
      // return: p, i, d, mino, maxo
      cmd->start_command(CMD_PID_CONFIG);
      cmd->add_arg(ji);
      cmd->add_arg(pid->get_p());
      cmd->add_arg(pid->get_i());
      cmd->add_arg(pid->get_d());
      cmd->add_arg(pid->get_output_min());
      cmd->add_arg(pid->get_output_max());
      cmd->finish_command();
      return;
    };
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
      default:
          return;
  }
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_PWM_LIMITS);
    cmd->add_arg(ji);
    cmd->add_arg(valve->get_extend_pwm_min());
    cmd->add_arg(valve->get_extend_pwm_max());
    cmd->add_arg(valve->get_retract_pwm_min());
    cmd->add_arg(valve->get_retract_pwm_max());
    cmd->finish_command();
    return;
  };
  int emin = cmd->get_arg<int>();
  if (!cmd->has_arg()) return;
  int emax = cmd->get_arg<int>();
  if (!cmd->has_arg()) return;
  int rmin = cmd->get_arg<int>();
  if (!cmd->has_arg()) return;
  int rmax = cmd->get_arg<int>();
  if (emax <= emin) return;
  if (rmax <= rmin) return;
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
      default:
          return;
  }
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_ADC_LIMITS);
    cmd->add_arg(ji);
    cmd->add_arg(pot->get_adc_min());
    cmd->add_arg(pot->get_adc_max());
    cmd->finish_command();
    return;
  };
  float amin = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float amax = cmd->get_arg<float>();
  if (amax <= amin) return;
  // disable pid (estop?)
  leg->disable_pids();
  pot->set_adc_range(amin, amax);
  //pid->reset();
  // re-enable pid? need to do this outside the loop
}

void on_calf_scale(CommandProtocol *cmd) {
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_CALF_SCALE);
    cmd->add_arg(leg->calf_load_transform->get_slope());
    cmd->add_arg(leg->calf_load_transform->get_offset());
    cmd->finish_command();
    return;
  };
  float s = cmd->get_arg<float>();
  if (!cmd->has_arg()) return;
  float o = cmd->get_arg<float>();
  leg->calf_load_transform->set_slope(s);
  leg->calf_load_transform->set_offset(o);
}

void on_report_time(CommandProtocol *cmd) {
  if (cmd->has_arg()) {
    report_time = cmd->get_arg<unsigned long>();
  } else {
    cmd->start_command(CMD_REPORT_TIME);
    cmd->add_arg((unsigned long)(report_time));
    cmd->finish_command();
  }
}

void on_report_adc(CommandProtocol *cmd) {
  bool enable = true;
  if (cmd->has_arg()) enable = cmd->get_arg<bool>();
  to_report.adc = enable;
}

void on_report_pwm(CommandProtocol *cmd) {
  bool enable = true;
  if (cmd->has_arg()) enable = cmd->get_arg<bool>();
  to_report.pwm = enable;
}

void on_report_pid(CommandProtocol *cmd) {
  bool enable = true;
  if (cmd->has_arg()) enable = cmd->get_arg<bool>();
  to_report.pid = enable;
}

void on_report_xyz(CommandProtocol *cmd) {
  bool enable = true;
  if (cmd->has_arg()) enable = cmd->get_arg<bool>();
  to_report.xyz = enable;
}

void on_report_angles(CommandProtocol *cmd) {
  bool enable = true;
  if (cmd->has_arg()) enable = cmd->get_arg<bool>();
  to_report.angles = enable;
}

void on_report_loop_time(CommandProtocol *cmd) {
  bool enable = true;
  if (cmd->has_arg()) enable = cmd->get_arg<bool>();
  to_report.loop_time = enable;
}

void on_pid_seed_time(CommandProtocol *cmd) {
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_PID_SEED_TIME);
    cmd->add_arg(leg->get_next_pid_seed_time());
    cmd->add_arg(leg->get_future_pid_seed_time());
    cmd->finish_command();
    return;
  }
  unsigned long next_time = cmd->get_arg<unsigned long>();
  if (!cmd->has_arg()) return;
  unsigned long future_time = cmd->get_arg<unsigned long>();
  leg->set_next_pid_seed_time(next_time);
  leg->set_future_pid_seed_time(future_time);
}

void on_reset_pids(CommandProtocol *cmd) {
  bool i_only = false;
  if (cmd->has_arg()) i_only = cmd->get_arg<bool>();
  if (i_only) {
    leg->reset_pids_i();
  } else {
    leg->reset_pids();
  }
}

void on_dither(CommandProtocol *cmd) {
  // joint index, time, amp
  if (!cmd->has_arg()) return;
  byte ji = cmd->get_arg<byte>();
  if (ji > 2) return;
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
      default:
          return;
  }
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_DITHER);
    cmd->add_arg(ji);
    cmd->add_arg(valve->get_dither_time());
    cmd->add_arg(valve->get_dither_amp());
    cmd->finish_command();
    return;
  };
  unsigned long dither_time = cmd->get_arg<unsigned long>();
  if (!cmd->has_arg()) return;
  int dither_amp = cmd->get_arg<int>();
  valve->set_dither_time(dither_time);
  valve->set_dither_amp(dither_amp);
}


void setup(){
  Serial.begin(9600);

  leg->estop->register_callback(send_estop);

  com.register_protocol(0, cmd);
  cmd.register_callback(CMD_ESTOP, on_estop);
  cmd.register_callback(CMD_HEARTBEAT, on_heartbeat);
  cmd.register_callback(CMD_PWM, on_pwm);
  cmd.register_callback(CMD_PLAN, on_plan);
  cmd.register_callback(CMD_ENABLE_PID, on_enable_pid);
  cmd.register_callback(CMD_PID_CONFIG, on_pid_config);
  cmd.register_callback(CMD_LEG_NUMBER, on_leg_number);
  cmd.register_callback(CMD_PWM_LIMITS, on_pwm_limits);
  cmd.register_callback(CMD_ADC_LIMITS, on_adc_limits);
  cmd.register_callback(CMD_CALF_SCALE, on_calf_scale);
  cmd.register_callback(CMD_REPORT_TIME, on_report_time);
  cmd.register_callback(CMD_PID_SEED_TIME, on_pid_seed_time);
  cmd.register_callback(CMD_RESET_PIDS, on_reset_pids);
  cmd.register_callback(CMD_DITHER, on_dither);
  cmd.register_callback(CMD_REPORT_ADC, on_report_adc);
  cmd.register_callback(CMD_REPORT_PWM, on_report_pwm);
  cmd.register_callback(CMD_REPORT_PID, on_report_pid);
  cmd.register_callback(CMD_REPORT_XYZ, on_report_xyz);
  cmd.register_callback(CMD_REPORT_ANGLES, on_report_angles);
  cmd.register_callback(CMD_REPORT_LOOP_TIME, on_report_loop_time);
}


void check_report() {
  if (report_timer > report_time) {
    /*
    cmd.start_command(CMD_LOOP_TIME);
    cmd.add_arg(t0);
    cmd.finish_command();
    */

    if (to_report.adc) {
      cmd.start_command(CMD_REPORT_ADC);
      cmd.add_arg(leg->hip_analog_sensor->get_adc_value());
      cmd.add_arg(leg->thigh_analog_sensor->get_adc_value());
      cmd.add_arg(leg->knee_analog_sensor->get_adc_value());
      cmd.add_arg(leg->calf_analog_sensor->get_adc_value());
      cmd.finish_command();
    }

    if (to_report.pid) {
      cmd.start_command(CMD_REPORT_PID);
      cmd.add_arg(leg->hip_joint->get_pid_output());
      cmd.add_arg(leg->thigh_joint->get_pid_output());
      cmd.add_arg(leg->knee_joint->get_pid_output());

      cmd.add_arg(leg->hip_pid->get_setpoint());
      cmd.add_arg(leg->thigh_pid->get_setpoint());
      cmd.add_arg(leg->knee_pid->get_setpoint());

      cmd.add_arg(leg->hip_pid->get_error());
      cmd.add_arg(leg->thigh_pid->get_error());
      cmd.add_arg(leg->knee_pid->get_error());

      cmd.finish_command();
    }

    if (to_report.pwm) {
      cmd.start_command(CMD_REPORT_PWM);
      cmd.add_arg(leg->hip_valve->get_pwm());
      cmd.add_arg(leg->thigh_valve->get_pwm());
      cmd.add_arg(leg->knee_valve->get_pwm());
      cmd.finish_command();
    }

    if (to_report.angles | to_report.xyz) {
      leg->compute_foot_position();
    }

    if (to_report.angles) {
      // angles
      cmd.start_command(CMD_REPORT_ANGLES);
      cmd.add_arg(leg->joint_angles.hip);
      cmd.add_arg(leg->joint_angles.thigh);
      cmd.add_arg(leg->joint_angles.knee);
      cmd.add_arg(leg->calf_load);
      cmd.add_arg(leg->foot_position_valid);
      cmd.finish_command();
    }

    if (to_report.xyz) {
      cmd.start_command(CMD_REPORT_XYZ);
      cmd.add_arg(leg->foot_position.x);
      cmd.add_arg(leg->foot_position.y);
      cmd.add_arg(leg->foot_position.z);
      cmd.finish_command();
    }

    report_timer = 0;
  }
}

void loop() {
  com.handle_stream();
  //unsigned long t0 = micros();
  leg->update();
  //t0 = micros() - t0;
  check_report();
}
