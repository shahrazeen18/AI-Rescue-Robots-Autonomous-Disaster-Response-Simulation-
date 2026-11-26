#include "rescue_request.h"

namespace rr {

bool Request::transitionAllowed(int from, int to) {
  switch (from) {
    case QS_NOT_REPORTED:
      return to == QS_PENDING || to == QS_FAILED;
    case QS_PENDING:
      return to == QS_ASSIGNED || to == QS_FAILED;
    case QS_ASSIGNED:
      return to == QS_IN_PROGRESS || to == QS_PENDING;
    case QS_IN_PROGRESS:
      return to == QS_COMPLETED || to == QS_PENDING;
    default:  // COMPLETED / FAILED are terminal
      return false;
  }
}

bool Request::transition(int to) {
  if (!transitionAllowed(state, to)) return false;
  state = to;
  return true;
}

}  
