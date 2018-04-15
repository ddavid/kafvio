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

#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>

/** 
 * \brief Enum to represent the type of a cone.
 */
//enum pos_t {YELLOW, BLUE, RED, BIG_RED};
/** \brief This is the base of the library. The object_t struct contains all
 * information necessary to do SLAM.
 *
 * Member:
 *    * `double`: distance to from the camera to the cone in [m]
 *    * `double`: angle to the cone in driving direction in [rad]
 *    * `double`: width of the cone in [m]
 *    * `int`:  type or color of the cone `YELLOW`, `BLUE`, 'RED' or 'BIG_RED'
 *    * `size_t`: Timestamp
 */
typedef struct { 
  double distance; 
  double angle; 
  double angle_yaw;
  double width; 
  int type; // 0 = yellow, 1 = blue, 2 = red, 3 = big red
  size_t timestamp; 
} object_t;


/*
 * \brief The object_list_t struct contains object_t structs within itself
 *
 * Elements:
 *    * `uint32_t`: size is the number of `object_t` elements uint32_t for being comaptible with windows
 *    * `objet_t`: element is a pointer to the first element in the list
 */
typedef struct {
  uint32_t     size;
  object_t * elements;
} object_list_t;


// Interface:

/**
 * the object__new method creates and returns an object_t struct and fills the 
 * members with default values
 */
object_t object__new(void);

/**
 * the object__init method changes the elements "type", "distance", "angle" and
 * "width" of a given object_t struct with given values
 */
void object__init(
  object_t *ob,
  double distance,
  double angle,
  double angle_yaw,
  double width,
  int type
);

/**
 * the object__copy method copies an existing object_t struct and returns it 
 * back
 */
object_t object__copy(object_t* other);

/**
 * the object__set_distance method changes the member 'distance' of an object_t
 * struct with a given value
 */
void object__set_distance(object_t *ob, double distance);

/**
 * the object__set_angle method changes the member 'angle' of an object_t
 * struct with a given value
 */
void object__set_angle(object_t *ob, double angle);

/**
 * the object__set_width method changes the member 'width' of an object_t
 * struct with a given value
 */
void object__set_width(object_t *ob, double width);

/**
 * the object__equals method returns true if all members of both object_t 
 * structs are equal; returns false if not
 */
uint8_t object__equals(object_t* ob1, object_t* ob2);

/**
 * the object__delete method frees the memory previously allocated for an 
 * object_t struct 
 * in this case redundant, since memory is NOT dynamically allocated on the 
 * stack and will be automatically deleted after it runs out of scope
 */
void object__delete(object_t * object);

/**
 * the object_list__new_default method creates and returns an object_list_t 
 * struct which contains one element/object_t struct with default values
 */
object_list_t *object_list__new_default(void);

/**
 * the object_list__new method creates and returns an object_list_t struct which 
 * contains object_t structs with default values, whereas the number of object_t 
 * structs depends on the the value of the variable size
 */
object_list_t *object_list__new(uint32_t size);

/**
 * the object_list__copy method creates and returns an object_list_t struct.
 * Therefore memory will be dynamically allocated and the content of each 
 * element of a given object_list_t struct will be copied.
 */
object_list_t* object_list__copy(object_list_t* other);

/**
 * the object_list__size method returns the number of object_t structs inside a 
 * given object_list_t struct
 */
uint32_t object_list__size(object_list_t *ob_list);

/**
 * the object_list__empty method returns true if a given object_list_t struct
 * has no members and therefor is empty; returns false if the given 
 * object_list_t struct has at least one member
 */
uint8_t object_list__empty(object_list_t *ob_list);

/**
 * the object_list__begin method returns the pointer to the first object_t
 * struct inside a given object_list_t struct
 */
object_t* object_list__begin(object_list_t *ob_list);

/**
 * the object_list__begin method returns the pointer to the last object_t
 * struct inside a given object_list_t struct
 */
object_t* object_list__end(object_list_t *ob_list);

/**
 * the object_list__push_back method appends an object_t struct with default
 * values at the end of an object_list_t struct and the needed memory on the 
 * stack will be reallocated
 */
void object_list__push_back(object_list_t *ob_list);

/**
 * the object_list__push_back_copy method appends a GIVEN object_t struct at the 
 * end of an object_list_t struct and the needed memory on the stack will be 
 * reallocated
 */
void object_list__push_back_copy(object_list_t *ob_list, object_t *ob);

/**
 * the object_list__push_front method appends an object_t struct with default
 * values at the beginning of an object_list_t struct and the needed memory on 
 * the stack will be reallocated
 */
void object_list__push_front(object_list_t *ob_list);

/**
 * the object_list__push_front_copy method appends a GIVEN object_t struct at 
 * the beginning of an object_list_t struct and the needed memory on the stack 
 * will be reallocated
 */

void object_list__push_front_copy(object_list_t *ob_list,object_t *ob);

/**
 * the object_list__delete method frees the memory previously allocated for an 
 * object_list_t struct 
 * in this case necessary, since memory is dynamically allocated on the stack 
 * and will not be automatically deleted
 */
void object_list__delete(object_list_t * ob_list);

/**
 * the object__delete_last_element method deletes last object_t struct of an 
 * object_list_t struct and the needed memory on the stack will be reallocated
 */
void object_list__delete_last_element(object_list_t *ob_list);

#endif  // OBJECT_SRC_OBJECT_H_

