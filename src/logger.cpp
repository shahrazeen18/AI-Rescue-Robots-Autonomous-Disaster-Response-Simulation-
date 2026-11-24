#include "logger.h"

#include <cstdarg>

using namespace std;

namespace rr {

bool Logger::open(const char* path) {
  close();
  f_ = fopen(path, "w");
  if (!f_) return false;
  fprintf(f_, "time,event,robot,request,object,value,detail\n");
  lines_ = 0;
  return true;
}

void Logger::close() {
  if (f_) {
    fclose(f_);
    f_ = nullptr;
  }
}

void Logger::log(int time, const char* event, int robot, int request,
                 int object, long long value, const char* fmt, ...) {
  if (!f_) return;
  char detail[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(detail, sizeof detail, fmt, ap);
  va_end(ap);
  fprintf(f_, "%d,%s,", time, event);
  if (robot >= 0) fprintf(f_, "%d", robot);
  fputc(',', f_);
  if (request >= 0) fprintf(f_, "%d", request);
  fputc(',', f_);
  if (object >= 0) fprintf(f_, "%d", object);
  fprintf(f_, ",%lld,%s\n", value, detail);
  ++lines_;
}

}  // namespace rr
