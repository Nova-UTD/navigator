/*
 * Package:   can_translation
 * Filename:  include/can_translation/float_reporter_params.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "can_translation/types.hpp"

namespace navigator {
namespace can_translation {

struct float_reporter_params {
  can_data_t input_min;
  can_data_t input_max;
  double output_min;
  double output_max;
  can_id_t message_id;
  uint8_t field_start_bit;
  uint8_t field_length_bits;
};

}
}
