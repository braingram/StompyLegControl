#include <comando.h>

// imu
#define ENABLE_IMU
#ifdef ENABLE_IMU
#include <NXPMotionSense.h>
#include <Wire.h>
#include <EEPROM.h>
#endif

// thermocouple
#define ENABLE_TEMP
#ifdef ENABLE_TEMP
#include <Adafruit_MAX31856.h>
#define TC_CS 10
#define TC_DI 11
#define TC_DO 12
#define TC_CLK 13
#endif


// 3000 / 1024.
#define ENABLE_PRESSURE
// pressure sensor resistor is 165 ohms
// gauge reads 0-3000 for 4-20mA
#define ADC_RES 16
#define PRESSURE_RESISTOR 165
float psi_min_ticks = 0;
float psi_per_tick = 0;
#define FEED_PRESSURE_PIN A0

#define ENABLE_RPM
#define ENGINE_RPM_PIN 21

Comando com = Comando(Serial);
CommandProtocol cmd = CommandProtocol(com);
TextProtocol text = TextProtocol(com);
//#define DEBUG_TEXT

#define NAME 0  // imu
byte name = NAME;

#define HEARTBEAT_TIMEOUT 1000  // ms
bool allow_reports = false;
elapsedMillis hb_timer;

#define CMD_NAME 0
#define CMD_HEARTBEAT 1
#define CMD_FEED_PRESSURE 2  // analog: float

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
  if (allow_reports) report_sensor();
  _timer = 0;
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
  float pressure = analogRead(_pin) - psi_min_ticks;
  pressure *= psi_per_tick;
  if (pressure < 0) pressure = 0;
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

#ifdef ENABLE_IMU
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
#endif

#ifdef ENABLE_TEMP
class TemperatureBodySensor : public BodySensor {
  public:
    TemperatureBodySensor(CommandProtocol *cmd, byte index, byte cs_pin);
    void report_sensor();

    Adafruit_MAX31856 *tc;
};

TemperatureBodySensor::TemperatureBodySensor(CommandProtocol *cmd, byte index, byte cs_pin):BodySensor(cmd, index) {
  tc = new Adafruit_MAX31856(cs_pin, TC_DI, TC_DO, TC_CLK);
  tc->begin();
  tc->setThermocoupleType(MAX31856_TCTYPE_K);
};


void TemperatureBodySensor::report_sensor() {
  //tc->readFault();
  //tc->readCJTemperature();
  _cmd->start_command(_index);
  _cmd->add_arg(tc->readThermocoupleTemperature());
  _cmd->finish_command();
};
#endif

// ------------------------------------------------


/*
AnalogBodySensor *test;

// callbacks can't be non-static member functions
void set_analog_body_sensor_period(CommandProtocol *cmd) {
  test->_set_period(cmd);
};
*/

#ifdef ENABLE_PRESSURE
PressureBodySensor *feed_pressure;

void set_feed_pressure_period(CommandProtocol *cmd) {
  feed_pressure->_set_period(cmd);
};
#endif

#ifdef ENABLE_RPM
RPMBodySensor *rpm;

void set_rpm_period(CommandProtocol *cmd) {
  rpm->_set_period(cmd);
};

void rpm_tick() {
  rpm->tick();
};
#endif

#ifdef ENABLE_IMU
IMUBodySensor *imu;

void set_imu_period(CommandProtocol *cmd) {
  imu->_set_period(cmd);
};
#endif

#ifdef ENABLE_TEMP
TemperatureBodySensor *feed_temp;

void set_feed_temp_period(CommandProtocol *cmd) {
  feed_temp->_set_period(cmd);
};
#endif

void on_name(CommandProtocol *cmd) {
  cmd->start_command(CMD_NAME);
  cmd->add_arg(name);
  cmd->finish_command();
};

void on_heartbeat(CommandProtocol *cmd) {
  hb_timer = 0;
  allow_reports = true;
};

void setup(){
  // setup pressure sensor
  analogReadResolution(ADC_RES);
  int psi_max_ticks = ((1 << ADC_RES) - 1);
  psi_min_ticks = psi_max_ticks * (0.004 / 0.02);
  psi_per_tick = 3000. / (psi_max_ticks - psi_min_ticks);

  //pinMode(6, OUTPUT);
  //digitalWrite(6, HIGH);
  Serial.begin(9600);
  //test = new AnalogBodySensor(&cmd, 1, 1);
  //cmd.register_callback(test->_index, set_analog_body_sensor_period);
  com.register_protocol(0, cmd);
  cmd.register_callback(CMD_NAME, on_name);
  cmd.register_callback(CMD_HEARTBEAT, on_heartbeat);
#ifdef ENABLE_PRESSURE
  feed_pressure = new PressureBodySensor(
    &cmd, CMD_FEED_PRESSURE, FEED_PRESSURE_PIN);
  cmd.register_callback(feed_pressure->_index, set_feed_pressure_period);
#endif
#ifdef ENABLE_RPM
  rpm = new RPMBodySensor(&cmd, CMD_ENGINE_RPM, ENGINE_RPM_PIN);
  cmd.register_callback(rpm->_index, set_rpm_period);
  // setup rpm interrupt
  attachInterrupt(ENGINE_RPM_PIN, rpm_tick, RISING);
#endif
#ifdef ENABLE_IMU
  imu = new IMUBodySensor(&cmd, CMD_IMU_HEADING);
  cmd.register_callback(imu->_index, set_imu_period);
#endif
#ifdef ENABLE_TEMP
  feed_temp = new TemperatureBodySensor(&cmd, CMD_FEED_OIL_TEMP, TC_CS);
  cmd.register_callback(feed_temp->_index, set_feed_temp_period);
#endif
}

void loop() {
  com.handle_stream();
  if (hb_timer >= HEARTBEAT_TIMEOUT) {
    // reset all reports
    allow_reports = false;
  };
  //test->check();
#ifdef ENABLE_PRESSURE
  feed_pressure->check();
#endif
#ifdef ENABLE_RPM
  rpm->check();
#endif
#ifdef ENABLE_IMU
  imu->check();
#endif
#ifdef ENABLE_TEMP
  feed_temp->check();
#endif
};
