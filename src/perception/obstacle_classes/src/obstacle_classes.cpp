/*
 * Package:   obstacle_classes
 * Filename:  obstacle_classes.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "../include/obstacle_classes/obstacle_classes.hpp"
#include <string>

namespace navigator{
namespace obstacle_classes{
std::string get_class_string(OBSTACLE_CLASS cls){
    switch (cls)
    {
        case OBSTACLE_CLASS::PEDESTRIAN:
            return "PEDESTRIAN";
            break;

        case OBSTACLE_CLASS::VEHICLE:
            return "VEHICLE";
            break;

        default:
            return "UNKNOWN CLASS";
            break;
    }
}
}   // namespace obstacle_classes
}   // namespace navigator
