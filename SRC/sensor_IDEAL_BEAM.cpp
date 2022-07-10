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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "types.h"
#include "globals.h"
#include "utils.h"

#include "control_sensors_actuators.h"

/* globals */

typedef struct sensor_state_t_t sensor_state_t;
struct sensor_state_t_t
{
	double sense_completed_in_s;
	double last_sensor_reading;

};

// 1 cm / second
// Means 5 cm takes 5 seconds
#define VELOCITY_IN_M_PER_S 0.01 
// 10 degrees / second IN Radians
// Means 90degrees takes 9 seconds
#define TURN_IN_DEGREES_PER_S 0.1745329352 


/*-------------------------------------------------------------------------
 * (function: )
 *-----------------------------------------------------------------------*/
void* sensor_function_IDEAL_BEAM(sensor_t *sensor, agent_t *agent, double current_time) 
{
	beam_sensor_t *return_sensor_reading;
	return_sensor_reading = (beam_sensor_t*)malloc(sizeof(beam_sensor_t));

	sensor_state_t* sensor_state;

	return_sensor_reading->in_m = 1;
	return_sensor_reading->new_data = TRUE;
#if 0
	if (sensor->initialized == FALSE)
	{	
		sensor->initialized = TRUE;

		sensor_state = (sensor_state_t*)malloc(sizeof(sensor_state_t));
		sensor_state->sense_completed_in_s = 0;

		/* store as memory */
		sensor->general_memory = (void*)sensor_state;
	}
	else
	{
		/* extract memory */
		sensor_state = (sensor_state_t*)(sensor->general_memory);
	}

	if (sensor_state->sense_completed_in_s < current_time)
	{
		sim_obj_t* object_in_view;

		/* next sense completed at */
		sensor_state->sense_completed_in_s = current_time + sensor->sim_time_computation_epoch_s;

		/* get current reading */
		object_in_view = find_closest_object_on_beam_projection(agent);

		return_sensor_reading->in_m = generate_sensor_reading(object_in_view);
		return_sensor_reading->new_data = TRUE;
		sensor_state->last_sensor_reading = return_sensor_reading->in_m;
	}
	else
	{
		return_sensor_reading->in_m = sensor_state->last_sensor_reading;
		return_sensor_reading->new_data = FALSE;
	}

	return (void*)return_sensor_reading;
#endif
}

sim_obj_t* find_closest_object_on_beam_projection(agent_t *agent_self, double x, double y, double angle)
{
	int i;

	for (i = 0; i < num_sim_objects; i++)
	{
		if (sim_objects[i]->type == OBJECT)
		{
			if (sim_objects[i]->object->type == CIRCLE)
			{
			}
			else if (sim_objects[i]->object->type == RECTANGLE)
			{
			}
		}
		else if (sim_objects[i]->type == AGENT)
		{
			if (sim_objects[i]->agent == agent_self)
				continue;
			else 
			{
#if 0
				agent_x = sim_objects[i]->agent->x;
				agent_y = sim_objects[i]->agent->y;
				/* assume robots only spheres */
				oassert(sim_objects[i]->agent->agent_group->shape->type == CIRCLE);
				agent_radius = sim_objects[i]->agent->agent_group->shape->radius;
			run_agent_control(sim_objects[i]->agent, current_time);
#endif
			}
		}
	}
}
