// logger.h - structured, machine-readable simulation record (CSV).
//
// Columns: time,event,robot,request,object,value,detail
//   time    simulation step the record belongs to
//   event   record type (REQUEST_REPORTED, ASSIGN, MOVE, ROBOT_STATE, ...)
//   robot   external robot id (empty if not applicable)
//   request external request id (empty if not applicable)
//   object  external connection / location id (empty if not applicable)
//   value   a number whose meaning depends on the event (cost, battery, ...)
//   detail  short free text (never contains commas)
#ifndef RR_LOGGER_H
#define RR_LOGGER_H

#include <cstdio>

namespace rr {

class Logger {
 public:
  Logger() : f_(nullptr), lines_(0) {}
  ~Logger() { close(); }
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  // Opens the file and writes the CSV header. Returns false on failure.
  bool open(const char* path);
  void close();
  bool enabled() const { return f_ != nullptr; }
  long lines() const { return lines_; }

  // Pass -1 for robot/request/object when not applicable (the column is left
  // empty). The value column is always written.
  void log(int time, const char* event, int robot, int request, int object,
           long long value, const char* fmt, ...)
      __attribute__((format(printf, 8, 9)));

 private:
  FILE* f_;
  long lines_;
};

}  

#endif
