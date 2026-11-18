#include "logger.h"

#include <cstdarg>

namespace rr {

bool Logger::open(const char* path) {
  close();
  f_ = std::fopen(path, "w");
  if (!f_) return false;
  std::fprintf(f_, "time,event,robot,request,object,value,detail\n");
  lines_ = 0;
  return true;
}

void Logger::close() {
  if (f_) {
    std::fclose(f_);
    f_ = nullptr;
  }
}

void Logger::log(int time, const char* event, int robot, int request,
                 int object, long long value, const char* fmt, ...) {
  if (!f_) return;
  char detail[256];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(detail, sizeof detail, fmt, ap);
  va_end(ap);
  std::fprintf(f_, "%d,%s,", time, event);
  if (robot >= 0) std::fprintf(f_, "%d", robot);
  std::fputc(',', f_);
  if (request >= 0) std::fprintf(f_, "%d", request);
  std::fputc(',', f_);
  if (object >= 0) std::fprintf(f_, "%d", object);
  std::fprintf(f_, ",%lld,%s\n", value, detail);
  ++lines_;
}

}  // namespace rr
