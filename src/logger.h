/*
	logger.h - Logging library.
	Created by Brett Graham, June 11, 2017.
	Released into the public domain -- so help you God.
*/

#ifndef LOGGER_H
#define LOGGER_H

#include "Arduino.h"


#define LOG_LEVEL_NOTSET 0
#define LOG_LEVEL_DEBUG 10
#define LOG_LEVEL_INFO 20
#define LOG_LEVEL_WARNING 30
#define LOG_LEVEL_ERROR 40
#define LOG_LEVEL_CRITICAL 50


class Logger {
  public:
    Logger();

    void log(String msg, byte level);

    void set_level(byte level);
    byte get_level();

    void set_writer(void (*writer)(String msg));
  private:
    byte _level;
    void (*_writer)(String msg);
};

extern Logger logger;

#endif
