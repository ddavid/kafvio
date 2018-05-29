// Copyright 2017 municHMotorsport e.V. <info@munichmotorsport.de>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and

#ifndef OBJECT_SRC_OBJECT_H_
#define OBJECT_SRC_OBJECT_H_
#define MAX_LISTSIZE 30

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>


/** 
 *
 * Member:
 *    * `double`: distance to from the camera to the cone in [m]
 *    * `double`: angle to the cone in driving direction in [rad]
 *    * `int`: type of object  0 - yellow, 1 - blue, 2 - little red, 3 - big red
 *    */
typedef struct { 
    double distance; 
    double angle; 
    int type;
} object_t;


/*
 * \brief The object_list_t struct contains object_t structs within itself
 *
 * Elements:
 *    * `uint32_t`: size is the number of `object_t` elements for being comaptible with windows
 *    * `object_t`: element is an array of `object_t` elements
 */
typedef struct {
	uint32_t     size;
	object_t  element[MAX_LISTSIZE];
} object_list_t;



#endif  // OBJECT_SRC_OBJECT_H_

