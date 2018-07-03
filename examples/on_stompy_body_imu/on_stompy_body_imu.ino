#include <comando.h>

// imu
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>

// thermocouple
#include <Adafruit_MAX31856.h>
#define TC_CS 10
#define TC_DI 11
#define TC_DO 12
#define TC_CLK 13


// 3000 / 1024.
#define PSI_PER_TICK 2.9296875
#define FEED_PRESSURE_PIN 1

#define ENGINE_RPM_PIN 0

Comando com = Comando(Serial);
CommandProtocol cmd = CommandProtocol(com);
TextProtocol text = TextProtocol(com);
//#define DEBUG_TEXT

#define NAME 0  // imu

#define CMD_NAME 0
#define CMD_FEED_PRESSURE 1  // analog: float

#define CMD_FEED_OIL_TEMP 5  // serial: float
#define CMD_RETURN_OIL_TEMP 6  // serial: float

#define CMD_IMU_HEADING 15  // imu, serial: float, float, float

#define CMD_ENGINE_RPM 21  // digital pulse per rev: float
// 2000 rpm, ~34 rev per sec, ~30 ms between pulses
// setup an interrupt, count # of millis between
// maybe average every 10 reads (iir)


class BodySensor {
  public:
    BodySensor(CommandProtocol *cmd, byte index);
    virtual void report_sensor() = 0;
    void check();

    void _set_period(CommandProtocol *cmd);
    byte period;

    CommandProtocol *_cmd;
    byte _index;
    elapsedMillis _timer;
}; 



BodySensor::BodySensor(CommandProtocol *cmd, byte index) {
  _index = index;
  _cmd = cmd;
  period = 0;
};

void BodySensor::_set_period(CommandProtocol *cmd) {
  if (!cmd->has_arg()) return;
  period = cmd->get_arg<byte>();
  _timer = 0;
};

void BodySensor::check() {
  if (period == 0) return;
  if (_timer < period) return;
  report_sensor();
};


class AnalogBodySensor : public BodySensor {
  public:
    AnalogBodySensor(CommandProtocol *cmd, byte index, byte pin);
    void report_sensor();

    byte _pin;
};

AnalogBodySensor::AnalogBodySensor(CommandProtocol *cmd, byte index, byte pin):BodySensor(cmd, index) {
  // setup analog pin
  _pin = pin;
};

void AnalogBodySensor::report_sensor() {
  int value = analogRead(_pin);
  _cmd->start_command(_index);
  _cmd->add_arg(value);
  _cmd->finish_command();
};

class PressureBodySensor : public BodySensor {
  public:
    PressureBodySensor(CommandProtocol *cmd, byte index, byte pin);
    void report_sensor();

    byte _pin;
};

PressureBodySensor::PressureBodySensor(CommandProtocol *cmd, byte index, byte pin):BodySensor(cmd, index) {
  // setup analog pin
  _pin = pin;
};

void PressureBodySensor::report_sensor() {
  float pressure = analogRead(_pin) * PSI_PER_TICK;
  _cmd->start_command(_index);
  _cmd->add_arg(pressure);
  _cmd->finish_command();
};

class RPMBodySensor : public BodySensor {
  public:
    RPMBodySensor(CommandProtocol *cmd, byte index, byte pin);
    void report_sensor();
    volatile void tick();

    elapsedMillis _tick_timer;
    byte _pin;
    volatile unsigned long _dt;
};

RPMBodySensor::RPMBodySensor(CommandProtocol *cmd, byte index, byte pin):BodySensor(cmd, index) {
  // setup analog pin
  _pin = pin;
  _tick_timer = 0;
};

volatile void RPMBodySensor::tick() {
  // attach to an interrupt
  _dt = _tick_timer;
  _tick_timer = 0;
};

void RPMBodySensor::report_sensor() {
  float rpm = _dt;
  if (rpm != 0) {
    rpm = 60000. / rpm;
  };
  _cmd->start_command(_index);
  _cmd->add_arg(rpm);
  _cmd->finish_command();
};


class IMUBodySensor : public BodySensor {
  public:
    IMUBodySensor(CommandProtocol *cmd, byte index);
    void report_sensor();
    void update_filter();
    void check();

    NXPMotionSense imu;
    NXPSensorFusion filter;
};

IMUBodySensor::IMUBodySensor(CommandProtocol *cmd, byte index):BodySensor(cmd, index) {
  imu.begin();
  filter.begin(100);
};

void IMUBodySensor::update_filter() {
  if (imu.available()) {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    // Read the motion sensors
    imu.readMotionSensor(ax, ay, az, gx, gy, gz, mx, my, mz);

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  };
};

void IMUBodySensor::check() {
  update_filter();
  this->BodySensor::check();
};

void IMUBodySensor::report_sensor() {
  _cmd->start_command(_index);
  _cmd->add_arg(filter.getRoll());
  _cmd->add_arg(filter.getPitch());
  _cmd->add_arg(filter.getYaw());
  _cmd->finish_command();
};

class TemperatureBodySensor : public BodySensor {
  public:
    TemperatureBodySensor(CommandProtocol *cmd, byte index, byte cs_pin);
    void report_sensor();

    Adafruit_MAX31856 *tc;
};

TemperatureBodySensor::TemperatureBodySensor(CommandProtocol *cmd, byte index, byte cs_pin):BodySensor(cmd, index) {
  tc = new Adafruit_MAX31856(cs_pin, TC_DI, TC_DO, TC_CLK);
};


void TemperatureBodySensor::report_sensor() {
  _cmd->start_command(_index);
  _cmd->add_arg(tc->readThermocoupleTemperature());
  _cmd->finish_command();
};

// ------------------------------------------------


/*
AnalogBodySensor *test;

// callbacks can't be non-static member functions
void set_analog_body_sensor_period(CommandProtocol *cmd) {
  test->_set_period(cmd);
};
*/

PressureBodySensor *feed_pressure;

void set_feed_pressure_period(CommandProtocol *cmd) {
  feed_pressure->_set_period(cmd);
};

RPMBodySensor *rpm;

void set_rpm_period(CommandProtocol *cmd) {
  rpm->_set_period(cmd);
};

void rpm_tick() {
  rpm->tick();
};

IMUBodySensor *imu;

void set_imu_period(CommandProtocol *cmd) {
  imu->_set_period(cmd);
};

TemperatureBodySensor *feed_temp;

void set_feed_temp_period(CommandProtocol *cmd) {
  feed_temp->_set_period(cmd);
};

void setup(){
  Serial.begin(9600);
  //test = new AnalogBodySensor(&cmd, 1, 1);
  //cmd.register_callback(test->_index, set_analog_body_sensor_period);
  feed_pressure = new PressureBodySensor(
    &cmd, CMD_FEED_PRESSURE, FEED_PRESSURE_PIN);
  rpm = new RPMBodySensor(&cmd, CMD_ENGINE_RPM, ENGINE_RPM_PIN);
  imu = new IMUBodySensor(&cmd, CMD_IMU_HEADING);
  feed_temp = new TemperatureBodySensor(&cmd, CMD_FEED_OIL_TEMP, TC_CS);

  cmd.register_callback(feed_pressure->_index, set_feed_pressure_period);
  cmd.register_callback(rpm->_index, set_rpm_period);
  cmd.register_callback(imu->_index, set_imu_period);
  cmd.register_callback(feed_temp->_index, set_feed_temp_period);

  // setup rpm interrupt
  attachInterrupt(ENGINE_RPM_PIN, rpm_tick, RISING);
}

void loop() {
  //test->check();
  feed_pressure->check();
  rpm->check();
  imu->check();
  feed_temp->check();
};
