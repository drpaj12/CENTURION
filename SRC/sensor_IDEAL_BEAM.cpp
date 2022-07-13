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
#include <math.h>

#include "types.h"
#include "globals.h"
#include "utils.h"

#include "control_sensors_actuators.h"
#include "sensors.h"

/* globals */

typedef struct sensor_state_t_t sensor_state_t;
struct sensor_state_t_t
{
	double sense_completed_in_s;
	beam_sensor_t *sensor_reading;
};

/*-------------------------------------------------------------------------
 * (function: )
 *-----------------------------------------------------------------------*/
void* sensor_function_IDEAL_BEAM(sensor_t *sensor, agent_t *agent, double current_time) 
{
	beam_sensor_t *sensor_reading;
	sensor_state_t* sensor_state;

	if (agent->sensor_memories[sensor->sensor_idx] == NULL)
	{	
		sensor_state = (sensor_state_t*)malloc(sizeof(sensor_state_t));
		sensor_state->sense_completed_in_s = 0;
		sensor_reading = (beam_sensor_t*)malloc(sizeof(beam_sensor_t));
		sensor_state->sensor_reading = sensor_reading;

		/* store as memory */
		agent->sensor_memories[sensor->sensor_idx] = (void*)sensor_state;
	}
	else
	{
		/* extract memory */
		sensor_state = (sensor_state_t*)(agent->sensor_memories[sensor->sensor_idx]);
		sensor_reading = sensor_state->sensor_reading;
	}

	if (sensor_state->sense_completed_in_s < current_time)
	{
		/* next sense completed at */
		sensor_state->sense_completed_in_s = current_time + sensor->sim_time_computation_epoch_s;

		/* get current reading */
		find_closest_object_on_beam_projection(&sensor_reading, agent, agent->circle->center.x, agent->circle->center.y, agent->circle->radius+.5, agent->angle);
		sensor_reading->new_data = TRUE;
	}
	else
	{
		sensor_reading->new_data = FALSE;
	}

	return (void*)sensor_reading;
}


