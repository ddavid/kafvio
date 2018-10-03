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

#include "object.hpp"


// create new Object ----------------------------------------------------------
object_t object__new(void) 
{
  object_t ob;

	ob.timestamp = 0;
	ob.type      = -1;
	ob.distance  = -1;
	ob.angle     = -361;
	ob.width     = -1;

	return ob;
}


// initialize an Object -------------------------------------------------------
void object__init(object_t * ob, double distance, double angle, double width, int type) 
{
	ob->timestamp = 0;
	ob->type      = type;
	ob->distance  = distance;
	ob->angle     = angle;
	ob->width     = width;
}


// copy an object -------------------------------------------------------------
object_t object__copy(object_t * other) 
{
  return *other;
}


// initialize the distance of an object ---------------------------------------
void object__set_distance(object_t *ob, double distance) 
{
	ob->timestamp = 0;
	ob->type      = 1;
	ob->distance  = distance;
}


// initialize the angle of an object ------------------------------------------
void object__set_angle(object_t *ob, double angle) 
{
	ob->timestamp = 0;
	ob->type      = 1;
	ob->angle     = angle;
}


// initialize the width of an object ------------------------------------------
void object__set_width(object_t *ob, double width) 
{
	ob->timestamp = 0;
	ob->type      = 1;
	ob->width     = width;
}


// compare two objects, return 1 if equal -------------------------------------
uint8_t object__equals(object_t* ob1, object_t* ob2) 
{
  if ((ob1->timestamp == ob2->timestamp) &&
      (ob1->type      == ob2->type)      &&
      (ob1->distance  == ob2->distance)  &&
      (ob1->angle     == ob2->angle)     &&
      (ob1->width     == ob2->width)
     ) 
       {return 1;} 
  
  else {return 0;}

}



// free memory for an object --------------------------------------------------
void object__delete(object_t *object) 
{
	free(object);			// Gibt reservierten Speicher von object frei
}


// new object_list ------------------------------------------------------------
object_list_t * object_list__new_default(void) 
{
	return object_list__new(1);
}


// resize object_list ---------------------------------------------------------
object_list_t * object_list__new(uint32_t size) 
{
	object_list_t * ob_list = (object_list_t*) malloc(sizeof(object_list_t));

	assert(ob_list !=NULL);

	ob_list->elements       = (object_t *) malloc(sizeof(object_t) * size);
	ob_list->size           = size;

	for(unsigned i = 0; i < ob_list -> size; ++i) 
  {
		object_t object       = object__new();
		ob_list->elements[i]  = object;
	}

	return ob_list;
}


// copy an object_list --------------------------------------------------------
object_list_t* object_list__copy(object_list_t* other) 
{
  object_list_t * ob_list_copy  = object_list__new(other->size);

  for(unsigned i = 0; i < (ob_list_copy->size); ++i) 
  {
    ob_list_copy->elements[i] = other->elements[i];
  }

  return ob_list_copy;
}


// give the number of elements in the object_list -----------------------------
uint32_t object_list__size(object_list_t *ob_list) 
{
  return ob_list->size;
}


// check if the object_list is empty ------------------------------------------
uint8_t object_list__empty(object_list_t *ob_list) 
{
  if (!ob_list->size) 

       {return 1;} 

  else {return 0;}
}


// return the pointer to the first object in the object_list ------------------
object_t* object_list__begin(object_list_t *ob_list) 
{
  return &(ob_list->elements[0]);
}


// return the pointer to the last object in the object_list -------------------
object_t * object_list__end(object_list_t *ob_list) 
{
  return &(ob_list->elements[(ob_list->size)-1]);
}


// push back a new object in object_list --------------------------------------
void object_list__push_back(object_list_t *ob_list) 
{
  ++(ob_list->size);

  ob_list->elements = (object_t *) realloc(ob_list->elements,
                                           sizeof(object_t) * ob_list->size);

  ob_list->elements[(ob_list -> size)-1] = object__new();
}


// push back a copy object in object_list -------------------------------------
void object_list__push_back_copy(object_list_t *ob_list, object_t *ob) 
{
  ++(ob_list->size);

  ob_list->elements = (object_t *) realloc(ob_list->elements,
                                           sizeof(object_t) * ob_list->size);

  ob_list->elements[(ob_list -> size)-1] = *ob;
}


// push one new object at the begin of the list -------------------------------
void object_list__push_front(object_list_t *ob_list)
{
  ++(ob_list->size);

  ob_list->elements = (object_t *) realloc(ob_list->elements,
                                           sizeof(object_t) * ob_list->size);


  for(unsigned i = (ob_list->size) - 1; i > 0 ; --i) 
  {
    ob_list->elements[i] = ob_list->elements[i-1];
  }

  object_t a           = object__new();
  ob_list->elements[0] = a;
}


// push an copy object at the begin of the list -------------------------------
void object_list__push_front_copy(object_list_t * ob_list, object_t * ob) 
{
  ++(ob_list->size);

  ob_list->elements = (object_t *) realloc(ob_list->elements,
                                           sizeof(object_t) * ob_list->size);

  for(unsigned i = (ob_list -> size)-1; i > 0; --i) 
  {
    ob_list->elements[i] = ob_list->elements[i-1];
  }

  ob_list->elements[0] = *ob;
}


// free the memory of an object -----------------------------------------------
void object_list__delete(object_list_t * ob_list) 
{
  free(ob_list);  // Deallocate the memory of the objectlist
}


// delete last object of an object_list ---------------------------------------
void object_list__delete_last_element(object_list_t *ob_list) 
{
  --(ob_list->size);

	ob_list->elements = (object_t *) realloc(ob_list->elements,
                                           sizeof(object_t) * ob_list->size);
}

