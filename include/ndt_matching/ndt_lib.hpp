#ifndef NDT_MATCHING__NDT_LIB_HPP_
#define NDT_MATCHING__NDT_LIB_HPP_

#include "ndt_matching/visibility_control.h"
#include <string>

namespace ndt_matching {

class NdtLib {
public:
  NdtLib(std::string message);

  virtual ~NdtLib();

  int run_ndt_matching();
};

} // namespace ndt_matching

#endif // NDT_MATCHING__NDT_LIB_HPP_
