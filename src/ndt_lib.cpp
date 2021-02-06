#include "ndt_matching/ndt_lib.hpp"

#include <iostream>

namespace ndt_matching {

NdtLib::NdtLib(std::string message) {
  std::cout << "ndt_lib constructed by node " << message << std::endl;
}

NdtLib::~NdtLib() {}

} // namespace ndt_matching
