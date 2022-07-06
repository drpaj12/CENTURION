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

#ifndef CONTROL_SENSORS_ACTUATORS_H
#define CONTROL_SENSORS_ACTUATORS_H

#include "types.h"

/* CONTROL Algorithms */
extern void control_algorithm_OVERLORD(agent_t *agent, double current_time);
extern void control_algorithm_BASIC_AVOID_ICRA(agent_t *agent, double current_time);
extern void control_algorithm_SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE(agent_t *agent, double current_time);

/* SENSORS */
extern void* sensor_function_IDEAL_BEAM(sensor_t *sensor, agent_t *agent, double current_time);
extern void* sensor_function_ULTRASONIC(sensor_t *sensor, agent_t *agent, double current_time);
extern void* sensor_function_IR(sensor_t *sensor, agent_t *agent, double current_time);

/* ACTUATORS */
extern void actuator_function_IDEAL_TWO_WHEEL(actuator_t *actuator, agent_t *agent, act_inputs_t *inputs, double current_time);
extern void actuator_function_TWO_WHEEL(actuator_t *actuator, agent_t *agent, act_inputs_t *inputs, double current_time);

typedef struct beam_sensor_t_t beam_sensor_t;
struct beam_sensor_t_t
{
	double in_m;
};

#endif

