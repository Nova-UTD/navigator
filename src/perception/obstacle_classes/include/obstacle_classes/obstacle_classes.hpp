/*
 * Package:   obstacle_classes
 * Filename:  obstacle_classes.hpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef OBSTACLE_CLASSES_HPP
#define OBSTACLE_CLASSES_HPP

#include <stdint.h>
#include <string>

namespace navigator
{
namespace obstacle_classes
{

enum OBSTACLE_CLASS : uint8_t { VEHICLE, PEDESTRIAN };

std::string get_class_string(OBSTACLE_CLASS cls);

}   // namespace obstacle_classes
}   // namespace navigator

#endif  // OBSTACLE_CLASSES_HPP
