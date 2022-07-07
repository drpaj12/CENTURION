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

#include "robot_movement.h"

/* globals */

enum movement {FORWARD, BACKWARDS, RIGHT, LEFT, STOPPED};

typedef struct actuator_state_t_t actuator_state_t;
struct actuator_state_t_t
{
	movement move_type;
	double last_instruction_time_start;
	double last_instruction_time_end;
	double m_per_epoch;
	double angle_per_epoch;
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
void actuator_function_IDEAL_TWO_WHEEL(actuator_t *actuator, agent_t *agent, act_inputs_t *inputs, double current_time) 
{
	actuator_state_t* actuator_state;

	printf("IDEAL_TWO WHEEL ACTUATOR called\n");

	if (actuator->initialized == FALSE)
	{	
		actuator->initialized = TRUE;

		actuator_state = (actuator_state_t*)malloc(sizeof(actuator_state_t));
		/* velocity is m/s and simulator epoch is a time smaller than seconds -> m/sim_epoch = VEL * sim_time_epoch */
		actuator_state->m_per_epoch = VELOCITY_IN_M_PER_S * environment.sim_time_computation_epoch_s;
		/* angle is rad/s and simulator epoch is a time smaller than seconds -> rad/sim_epoch = RAD * sim_time_epoch */
		actuator_state->angle_per_epoch = TURN_IN_DEGREES_PER_S * environment.sim_time_computation_epoch_s;
		actuator_state->last_instruction_time_start = 0;
		actuator_state->last_instruction_time_end = 0;

		/* store as memory */
		actuator->general_memory = (void*)actuator_state;
	}
	else
	{
		/* extract memory */
		actuator_state = (actuator_state_t*)(actuator->general_memory);
	}

	if (inputs->new_instruction == TRUE)
	{
		/* record time of instruction and when should end */
		actuator_state->last_instruction_time_start = current_time;
		actuator_state->last_instruction_time_end = current_time + inputs->time_in_s;

		/* get direction */
		if (inputs->left == 1 && inputs->right == 1)
		{
			actuator_state->move_type = FORWARD;
		}
		else if (inputs->left == -1 && inputs->right == -11)
		{
			actuator_state->move_type = BACKWARDS;
		}
		else if (inputs->left == 0 && inputs->right == 0)
		{
			actuator_state->move_type = STOPPED;
		}
		else if (inputs->left == 1 && inputs->right == 0)
		{
			actuator_state->move_type = LEFT;
		}
		else if (inputs->left == 0 && inputs->right == 1)
		{
			actuator_state->move_type = RIGHT;
		}
		else 
		{
			printf("Actuator unsupported movement\n");
			exit(-1);
		}
	}

	/* check to see if instruction is completed */
	if (current_time < actuator_state->last_instruction_time_end)
	{
		switch (actuator_state->move_type)
		{
			case FORWARD:
				move(agent, actuator_state->m_per_epoch);
				break;
			case BACKWARDS:
				move(agent, -(actuator_state->m_per_epoch));
				break;
			case LEFT: // counter clock wise
				turn(agent, -(actuator_state->angle_per_epoch));
				break;
			case RIGHT: // clock wise
				turn(agent, actuator_state->angle_per_epoch);
				break;
			default:
				oassert(FALSE);
		}
	}
}
