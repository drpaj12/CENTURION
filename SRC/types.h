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
enum sensor_type {ULTRASONIC, IR};
struct sensor_t_t 
{
	sensor_type type;
	double angle; // <!-- assume 0 = 0 degrees facing forward of robot -->
	double sim_time_computation_epoch_s; // how fast the sensor reads

	double (*fptr_sensor)(agent_t *agent);
};

/* inputs to an actuator */
struct act_inputs_t_t 
{
	double left_v;
	double right_v;
};
	
/* different types of actuators */
enum actuator_type {TWO_WHEELS};
struct actuator_t_t 
{
	actuator_type type;
	double sim_time_computation_epoch_s; // how fast the sensor reads

	void (*fptr_actuator)(agent_t *agent, act_inputs_t *values);
};

/* different types of agent */
struct agent_t_t 
{
	agent_group_t *agent_group; // back pointer to description of agent

	/* personal state */
	double x;
	double y;
	double angle; // assuming in degrees where 0 degrees is North
	
	void *general_memory;

	/* personal goals */
};

enum shape_type {SPHERE, QUADRILATERAL};
/* the objects in the environment - sphere or quadrilateral */
struct objects_t_t 
{
	shape_type type;
	double radius;
	double x1;
	double y1;
	double x2;
	double y2;
	double x3;
	double y3;
	double x4;
	double y4;
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
	void (*control_algorithm)(agent_t *agent, double current_time);

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
	char *sim_log_file_out;
	char *simulation_type;
};

/* the environment - what the 2D space looks like */
struct environment_t_t 
{
	double real_size_x_in_m;
	double real_size_y_in_m;
	double sim_grid_size_x_in_m;
	double sim_grid_size_y_in_m;
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

#endif // TYPES_H
