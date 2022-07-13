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
#include "sensors.h"
#include "actuators.h"

/* globals */

/* Where the sensors and actuators are read in from in terms of data structure */
#define IDEAL_BEAM_SENSOR 0
#define IDEAL_TWO_WHEEL 0

// 10s * 1cm/s = 10cm
#define FORWARD_TIME 10
// 10degrees * 9s = 90 degrees
#define TURN_TIME 9

/*-------------------------------------------------------------------------
 * (function:control_algorithm_SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE )
 * 	Assume this robot has ideal_two_wheel acuator and an ideal beam sensor
 * 	The control goes in a square where 3s for 3cm and 9s for 90 degree turn
 * 	Simulation shows it's off by a bit
 *-----------------------------------------------------------------------*/
void control_algorithm_SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE(agent_t *agent, double current_time) 
{
	void *sensor_val;
	beam_sensor_t *sensor_data;
	int *mem_old_STATE;
	act_inputs_t actuator_input;

	/* with STATE being very big - S_START is STATE 0 */
	enum states {S_START, S_START_FORWARD, S_FORWARD, S_START_TURN_LEFT, S_TURN_LEFT, S_STOP_OBJECT};

	printf("SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE called\n");

	/* read sensor two check if something is 5cm away */
	sensor_val = run_sensor(agent->agent_group->sensors[IDEAL_BEAM_SENSOR], agent, current_time);
	sensor_data = (beam_sensor_t*)sensor_val;

	printf("sensor reads %f meters\n", sensor_data->in_m);

	/* sensor reads -1 if no objects */
	if (sensor_data->in_m > 0.0 && sensor_data->in_m < 0.05)
	{
		if (agent->CURRENT_STATE != S_START)
		{	
			/* if in 5cm of an object need to stop and store current state */
			((int*)(agent->general_memory))[0] = agent->CURRENT_STATE;
		       	agent->CURRENT_STATE = S_STOP_OBJECT;	
			/* tell robot to stop */
			actuator_input.left = 0;
			actuator_input.right = 0;
			actuator_input.time_in_s = 0;
			actuator_input.new_instruction = FALSE;
		}
	}
	else
	{
		switch (agent->CURRENT_STATE)
		{
			case S_START: 
				/* create memory in the robot */
				mem_old_STATE = (int*)malloc(sizeof(int));
				agent->general_memory = (void*)mem_old_STATE;
				agent->time_in_state = 0;	

				/* actuator inputs */
				actuator_input.left = 0;
				actuator_input.right = 0;
				actuator_input.new_instruction = FALSE;

				agent->CURRENT_STATE = S_START_FORWARD;
				break;
			case S_START_FORWARD: 
				agent->time_in_state = 0;	

				/* actuator inputs */
				actuator_input.left = 1;
				actuator_input.right = 1;
				actuator_input.time_in_s = FORWARD_TIME; // s at 1cm/s = 10cm
				actuator_input.new_instruction = TRUE;

				agent->CURRENT_STATE = S_FORWARD;
				break;
			case S_FORWARD: 
				agent->time_in_state += current_time - agent->last_time;	

				/* actuator inputs */
				actuator_input.new_instruction = FALSE;

				if (agent->time_in_state < FORWARD_TIME)
					agent->CURRENT_STATE = S_FORWARD;
				else
					agent->CURRENT_STATE = S_START_TURN_LEFT;
				break;
			case S_START_TURN_LEFT: 
				agent->time_in_state = 0;	

				/* actuator inputs */
				actuator_input.left = 1;
				actuator_input.right = 0;
				actuator_input.time_in_s = TURN_TIME; // 9s at 10degrees per second = 90 degrees
				actuator_input.new_instruction = TRUE;

				agent->CURRENT_STATE = S_TURN_LEFT;
				break;
			case S_TURN_LEFT: 
				agent->time_in_state += current_time - agent->last_time;	

				/* actuator inputs */
				actuator_input.new_instruction = FALSE;

				if (agent->time_in_state < TURN_TIME)
					agent->CURRENT_STATE = S_TURN_LEFT;
				else
					agent->CURRENT_STATE = S_START_FORWARD;
				break;
			case S_STOP_OBJECT:
				/* if back here then object no longer 5cm away */

				/* actuator inputs */
				actuator_input.new_instruction = FALSE;

				agent->CURRENT_STATE = ((int*)(agent->general_memory))[0];
				break;
			default:
				printf("Robot in unknown state\n");
				oassert(FALSE);
				break;
		}
	}

	/* move actuator */
	run_actuator( agent->agent_group->actuators[IDEAL_TWO_WHEEL], agent, &(actuator_input), current_time);

	printf("Robot at location x=%f, y=%f, angle=%f (degrees=%f)\n", agent->circle->center.x, agent->circle->center.y, agent->angle, agent->angle * (180.0 / PI));

	/* record last time for tracking details */
	agent->last_time = current_time;

	return;
}

