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
        case OBSTACLE_CLASS::PERSON:
            return "PERSON";
            break;

        case OBSTACLE_CLASS::BICYCLE:
            return "BICYCLE";
            break;
        case OBSTACLE_CLASS::CAR:
            return "CAR";
            break;

        case OBSTACLE_CLASS::MOTORBIKE:
            return "MOTORBIKE";
            break;

        case OBSTACLE_CLASS::BUS:
            return "BUS";
            break;

        case OBSTACLE_CLASS::TRUCK:
            return "TRUCK";
            break;

        default:
            return "OTHER";
            break;
    }
}
}   // namespace obstacle_classes
}   // namespace navigator
