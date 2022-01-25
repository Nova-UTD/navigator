/*
 * Package:   bridge
 * Filename:  bridging.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <string>
#include <vector>

namespace navigator {
  namespace bridge {
    std::string construct(std::vector<std::array<std::string, 2>> list);
    void log(std::string message);
  }
}
