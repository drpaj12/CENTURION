/*
Copyright (c) 2022 Peter Jamieson (jamieson.peter@gmail.com)
and Bryan Van Scoy

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/ 

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define PI    3.1415926359
#define twoPI 6.2831853072

#include <stdlib.h>

#include "argparse_value.hpp"

#ifndef TYPES_H
#define TYPES_H

//#define oassert(x) {if(!(x)){exit(-1);}} // causes an interrupt in GDB
#define oassert(x) {if(!(x)){__asm("int3");}} // causes an interrupt in GDB

typedef struct global_args_t_t global_args_t;
typedef struct sim_obj_t sim_obj_t;
typedef struct sensor_t_t sensor_t;
typedef struct act_inputs_t_t act_inputs_t;
typedef struct actuator_t_t actuator_t;
typedef struct agent_group_t_t agent_group_t;
typedef struct agent_t_t agent_t;
typedef struct objects_t_t objects_t;
typedef struct agent_groups_t_t agent_groups_t;
typedef struct sim_system_t_t sim_system_t;
typedef struct environment_t_t environment_t;
typedef struct communication_stack_t_t communication_stack_t;
typedef struct atons_t_t atons_t;
/* SHAPES in simulation */
typedef struct vector_2D_t_t vector_2D_t;
typedef struct line_t_t line_t;
typedef struct line_segment_t_t line_segment_t;
typedef struct rectangle_t_t rectangle_t;
typedef struct oriented_rectangle_t_t oriented_rectangle_t;
typedef struct circle_t_t circle_t;
typedef struct range_t_t range_t;
typedef struct points_t_t points_t;



/* the global arguments of the software */
struct global_args_t_t
{
	std::string program_name;
	argparse::ArgValue<char*> output_file;
	argparse::ArgValue<char*> test_file;
	argparse::ArgValue<char*> config_file;
	argparse::ArgValue<int> mode;
	argparse::ArgValue<bool> show_help;
};

enum sim_obj_type {OBJECT, AGENT};
/* the objects in the environment - sphere or quadrilateral */
struct sim_obj_t 
{
	sim_obj_type type;
	agent_t *agent;
	objects_t *object;
};

/* different types of sensor */
/* ASSUMPTION - sensor mounted at very front of robot, Radius of robot */
struct sensor_t_t 
{
	double angle; // <!-- assume 0 = 0 radian angle facing forward of robot, 1.57079 is facing East, and 3.14 is facing backwards (counter clockwise rotation) --> 
	double sim_time_computation_epoch_s; // how fast the sensor reads

	/* the function that returns void * data for what sensor sees */
	void* (*fptr_sensor)(sensor_t *sensor, agent_t *agent, double current_time);

	int sensor_idx;
};

/* inputs to an actuator */
struct act_inputs_t_t 
{
	double left;
	double right;
	double time_in_s;
	short new_instruction;
};
	
/* different types of actuators */
struct actuator_t_t 
{
	void (*fptr_actuator)(actuator_t *actuator, agent_t *agent, act_inputs_t *values, double current_time);

	/* actuator internal sim state */
	void *general_memory;
	short initialized;
};

/* different types of agent */
struct agent_t_t 
{
	agent_group_t *agent_group; // back pointer to description of agent
	short not_physical_agent; // for overlords and other agents of this type

	/* personal state */
	double angle; // assuming in radians where 0 degrees is East and West is "pi" = 3.14
	circle_t *circle;
	
	void *general_memory;
	void **sensor_memories;
	void **actuator_memories;

	int CURRENT_STATE;
	double time_in_state;
	double last_time;
	

	/* personal goals */
};

enum shape_type {CIRCLE, RECTANGLE};
/* the objects in the environment - sphere or quadrilateral */
struct objects_t_t 
{
	shape_type type;
	circle_t *circle;
	oriented_rectangle_t *rectangle;
};

/* different types of agent groups */
struct agent_group_t_t 
{
	char* initialization_function;
	int num_agents;
	agent_t **agents;

	/* robots can only be spheres right now */
	objects_t *shape;

	int num_sensors;
	sensor_t **sensors;
	int num_actuators;
	actuator_t **actuators;

	/* function pointer of control algorithm */
	void (*fptr_control_algorithm)(agent_t *agent, double current_time);

	// goals not here 
};

/* different types of agent groups */
struct agent_groups_t_t 
{
	int num_agent_groups;
	agent_group_t **agent_group;
};

/* the system file */
struct sim_system_t_t 
{
	int rand_seed;
	char *debug_file_out;
	FILE *Fdebug_out;
	char *sim_log_file_out;
	FILE *Fsim_log_out;
	int output_log_tab_step;
	char *simulation_type;
};

/* the environment - what the 2D space looks like */
struct environment_t_t 
{
	double real_size_x_in_m;
	double real_size_y_in_m;
	double sim_time_s; 
	double sim_time_computation_epoch_s; // assume the use has set this time to the smallest and all other sim_time are divisible by
	short boundary_walls;
	objects_t **objects;
	int num_objects;
};

/* the communications */
struct communication_stack_t_t 
{
	void *data;
};

/* the aids to navigation */
struct atons_t_t 
{
	void *data;
};

typedef struct str_t {
	int len, alloc;
	unsigned char *s;
} bstr_t, *bstr;

struct vector_2D_t_t 
{
	double x;
	double y;
};

/* line is infinite */
/* base is point where it starts, direction is vector it goes out on */
struct line_t_t
{
	vector_2D_t base;
	vector_2D_t direction;
};
/* line segment is point1 and point2 */
struct line_segment_t_t
{
	vector_2D_t point1;
	vector_2D_t point2;
};

/* axis oreinted rectangle */
struct rectangle_t_t
{
	vector_2D_t origin;
	vector_2D_t size;
};

/* oreinted rectangle */
struct oriented_rectangle_t_t
{
	vector_2D_t center;
	vector_2D_t halfExtend;
	double rotation;
};

/* axis oreinted rectangle */
struct circle_t_t
{
	vector_2D_t center;
	double radius;
};

struct range_t_t
{
	double minimum;
	double maximum;
};

struct points_t_t
{
	int num_points;
	vector_2D_t **points;
};

#endif // TYPES_H
