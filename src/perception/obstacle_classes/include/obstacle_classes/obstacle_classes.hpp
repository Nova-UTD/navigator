/*
 * Package:   obstacle_drawer
 * Filename:  obstacle_drawer.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef OBSTACLE_CLASSES_HPP
#define OBSTACLE_CLASSES_HPP

#include <stdint.h>
#include <string>

namespace navigator {
    namespace obstacle_classes {

        enum OBSTACLE_CLASS : uint8_t { VEHICLE, PEDESTRIAN };

        std::string get_class_string(OBSTACLE_CLASS cls);

    }
}

#endif