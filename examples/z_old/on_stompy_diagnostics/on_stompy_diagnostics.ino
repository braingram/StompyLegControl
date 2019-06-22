// has stompy pins
#include "leg.h"

#include <comando.h>

Comando com = Comando(Serial);
CommandProtocol cmd = CommandProtocol(com);
TextProtocol text = TextProtocol(com);
//#define DEBUG_TEXT


Estop estop = Estop();
byte last_estop_severity = 255;
byte joint = 255;  // -1 = undefined, 0,1,2: hip,thigh,knee
float pwm = 0;

// 10 ms = 100 Hz
unsigned long report_time = 10;
elapsedMillis report_adc_timer;

int extend_pins[] = {
  HIP_EXTEND_PIN, THIGH_EXTEND_PIN, KNEE_EXTEND_PIN};
int retract_pins[] = {
  HIP_RETRACT_PIN, THIGH_RETRACT_PIN, KNEE_RETRACT_PIN};
int enable_pins[] = {
  HIP_ENABLE_PIN, THIGH_ENABLE_PIN, KNEE_ENABLE_PIN};
int sensor_pins[] = {
  HIP_SENSOR_PIN, THIGH_SENSOR_PIN, KNEE_SENSOR_PIN};

#define CMD_HEARTBEAT 0

#define CMD_ESTOP 1

#define CMD_JOINT 2
#define CMD_PWM 3
#define CMD_PWM_RAMP 4

#define CMD_REPORT_ADC 21
#define CMD_REPORT_PRESSURE 22


void on_heartbeat(CommandProtocol *cmd) {
  estop.set_heartbeat();
  //leg->estop->set_heartbeat();
#ifdef DEBUG_TEXT
  text.print("hb\n");
#endif
  // send heartbeat back
  cmd->start_command(CMD_HEARTBEAT);
  cmd->finish_command();
};


void on_estop(CommandProtocol *cmd){
  byte severity = ESTOP_DEFAULT;
  if (cmd->has_arg()) {
    severity = cmd->get_arg<byte>();
    estop.set_estop(severity);
  } else {
    cmd->start_command(CMD_ESTOP);
    cmd->add_arg((byte)(estop.get_estop()));
    cmd->finish_command();
  }
};


void enable_valves() {
  digitalWrite(DISABLE_PIN, LOW);
  for (int i=0; i<3; i++) {
    digitalWrite(enable_pins[i], HIGH);
  };
};


void disable_valves() {
  digitalWrite(DISABLE_PIN, HIGH);
  for (int i=0; i<3; i++) {
    digitalWrite(enable_pins[i], LOW);
  };
};


void estop_callback(byte severity) {
  // enable/disable valves
  if (estop.is_stopped()) {
    disable_valves();
  } else {
    enable_valves();
  };
  // report to cpu
  if (severity != last_estop_severity) {
    cmd.start_command(CMD_ESTOP);
    cmd.add_arg((byte)(severity));
    cmd.finish_command();
    last_estop_severity = severity;
  };
};

void set_joint(byte new_joint) {
  if (new_joint > 2) return;
  if (joint != new_joint) {
    joint = new_joint;
    cmd.start_command(CMD_JOINT);
    cmd.add_arg((byte)(joint));
    cmd.finish_command();
  };
};

void on_joint(CommandProtocol *cmd) {
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_JOINT);
    cmd->add_arg((byte)(joint));
    cmd->finish_command();
    return;
  } else {
    set_joint(cmd->get_arg<byte>());
  };
};

void set_joint_pwm(byte joint, float pwm) {
  if (joint > 2) return;
  if (pwm < -1) return;
  if (pwm > 1) return;
  // compute pwm
  if (pwm > 0) {
    analogWrite(retract_pins[joint], 0);
    analogWrite(extend_pins[joint], 0);
  } else {
    analogWrite(extend_pins[joint], 0);
    analogWrite(retract_pins[joint], 0);
  };
};

void set_pwm(float new_pwm) {
  if ((new_pwm < -1) || (new_pwm > 1)) return;
  if (pwm != new_pwm) {
    // 
    estop.check_heartbeat();
    if (estop.is_stopped()) new_pwm = 0.; 
    pwm = new_pwm;
    set_joint_pwm(joint, pwm);
    cmd.start_command(CMD_PWM);
    cmd.add_arg((float)(pwm));
    cmd.finish_command();
  };
};

void on_pwm(CommandProtocol *cmd) {
  if (!cmd->has_arg()) {
    cmd->start_command(CMD_PWM);
    cmd->add_arg((float)(pwm));
    cmd->finish_command();
    return;
  } else {
    set_pwm(cmd->get_arg<float>());
  };
};


void report_adcs(){
}

void setup() {
  // disable pins, setup ADCs
  pinMode(DISABLE_PIN, OUTPUT);
  digitalWrite(DISABLE_PIN, HIGH);
  for (int i=0; i<3; i++) {
    pinMode(enable_pins[i], OUTPUT);
    digitalWrite(enable_pins[i], LOW);
    analogWriteResolution(extend_pins[i], VALVE_PWM_RESOLUTION);
    analogWriteResolution(retract_pins[i], VALVE_PWM_RESOLUTION);
    analogWriteFrequency(extend_pins[i], VALVE_PWM_FREQUENCY);
    analogWriteFrequency(retract_pins[i], VALVE_PWM_FREQUENCY);
  };
  Serial.begin(9600);
  // TODO register commands
  estop.register_callback(estop_callback);
}


void loop() {
  estop.check_heartbeat();
  com.handle_stream();
  // TODO check report timer(s)
}
