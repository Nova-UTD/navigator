/*
 * Package:   obstacle_drawer
 * Filename:  obstacle_classes.hpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */



#ifndef OBSTACLE_CLASS_HPP
#define OBSTACL_CLASS_HPP

#include <stdint.h>

namespace navigator {

    namespace obstacle_repub {

        enum OBSTACLE_CLASS : uint8_t { 
            VEHICLE, 
            PEDESTRIAN
        };    

    }
    
}

#endif