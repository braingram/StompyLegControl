#include "logger.h"

Logger::Logger() {
  _level = LOG_LEVEL_DEBUG;
  _writer = NULL;
};

void Logger::log(String msg, byte level) {
  if (*_writer == NULL) return;
  if (level < _level) return;
  _writer(msg);
};

void Logger::set_level(byte level) {
  _level = level;
};

byte Logger::get_level() {
  return _level;
};

void Logger::set_writer(void (*writer)(String msg)) {
  _writer = writer;
};

Logger logger = Logger();
