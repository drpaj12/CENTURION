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

#include "robot_movement.h"
#include "collision_detection.h"

/* globals */

double turn_angle_in_seconds(double s);
double go_forward_distance_in_seconds(double s);
double go_forward_result_angle_in_s(double s); 

enum movement {FORWARD, BACKWARDS, RIGHT, LEFT, STOPPED};

typedef struct actuator_state_t_t actuator_state_t;
struct actuator_state_t_t
{
	movement move_type;
	double last_instruction_time_start;
	double last_instruction_time_end;
	double m_per_epoch;
	double drift_angle_per_epoch;
	double angle_per_epoch;
};

/*-------------------------------------------------------------------------
 * (function: )
 *-----------------------------------------------------------------------*/
void actuator_function_TWO_WHEEL(actuator_t *actuator, agent_t *agent, act_inputs_t *inputs, double current_time) 
{
	actuator_state_t* actuator_state;

	//printf("IDEAL_TWO WHEEL ACTUATOR called\n");

	if (agent->actuator_memories[actuator->actuator_idx] == NULL)
	{	
		actuator_state = (actuator_state_t*)malloc(sizeof(actuator_state_t));
		/* velocity is m/s and simulator epoch is a time smaller than seconds so use characterization for epoch */
		actuator_state->m_per_epoch = go_forward_distance_in_seconds(environment.sim_time_computation_epoch_s);
		actuator_state->drift_angle_per_epoch = go_forward_result_angle_in_s(environment.sim_time_computation_epoch_s);
		/* angle is rad/s and simulator epoch is a time smaller than seconds so use characterization for epoc */
		actuator_state->angle_per_epoch = turn_angle_in_seconds(environment.sim_time_computation_epoch_s);

		actuator_state->last_instruction_time_start = 0;
		actuator_state->last_instruction_time_end = 0;

		/* store as memory */
		agent->actuator_memories[actuator->actuator_idx] = (void*)actuator_state;
	}
	else
	{
		/* extract memory */
		actuator_state = (actuator_state_t*)(agent->actuator_memories[actuator->actuator_idx]);
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
		/* UPDATE characterization of robot */
		/* velocity is m/s and simulator epoch is a time smaller than seconds so use characterization for epoch */
		actuator_state->m_per_epoch = go_forward_distance_in_seconds(environment.sim_time_computation_epoch_s);
		actuator_state->drift_angle_per_epoch = go_forward_result_angle_in_s(environment.sim_time_computation_epoch_s);
		/* angle is rad/s and simulator epoch is a time smaller than seconds so use characterization for epoc */
		actuator_state->angle_per_epoch = turn_angle_in_seconds(environment.sim_time_computation_epoch_s);
		printf("epochs: %f, m/s:%f, drift:%f, angle:%f\n", environment.sim_time_computation_epoch_s, actuator_state->m_per_epoch, actuator_state->drift_angle_per_epoch, actuator_state->angle_per_epoch);

		switch (actuator_state->move_type)
		{
			case FORWARD:
				move_with_drift(agent, actuator_state->m_per_epoch, actuator_state->drift_angle_per_epoch);
				break;
			case BACKWARDS:
				move_with_drift(agent, -actuator_state->m_per_epoch, actuator_state->drift_angle_per_epoch);
				break;
			case LEFT: // counter clock wise = adding
				turn(agent, actuator_state->angle_per_epoch);
				break;
			case RIGHT: // clock wise = sub
				turn(agent, -(actuator_state->angle_per_epoch));
				break;
			default:
				oassert(FALSE);
		}
	}
}

/* GAUSSIAN code from
 * https://kcru.lawsonresearch.ca/research/srk/normalDBN_random.html
 */
double turn_angle_in_seconds(double s) 
{
	double mu = 237.5288752 * s + 19.28566864;
	double sigma = 3;
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;
	
	if (call == 1)
	{
		call = !call;
	//	return (mu + sigma * (double)X2);
        	return (degrees_to_radian(mu + sigma * (double)X1));
	}

	do
	{
		U1 = -1 + ((double)rand() / RAND_MAX) * 2;
		U2 = -1 + ((double)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	mult = sqrt((-2 * log(W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

//	call = !call;

	//return (mu + sigma * (double)X1);
        return (degrees_to_radian(mu + sigma * (double)X1));
}

double go_forward_distance_in_seconds(double s) 
{
	double mu = 22.660707 * s + 1.3405;
	double sigma = 0.15;
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;
	
	if (call == 1)
	{
		call = !call;
	//	return (mu + sigma * (double)X2);
		return ((mu + sigma * (double)X2)/100);
	}

	do
	{
		U1 = -1 + ((double)rand() / RAND_MAX) * 2;
		U2 = -1 + ((double)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	mult = sqrt((-2 * log(W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

//	call = !call;

	//return (mu + sigma * (double)X1);
	// In meters
	return ((mu + sigma * (double)X1)/100);
}

double go_forward_result_angle_in_s(double s) 
{
        double mu = 4.38 * s + 1.3874;
        double sigma = 2;	
        double U1, U2, W, mult;
        static double X1, X2;
        static int call = 0;

        if (call == 1)
        {
                call = !call;
//                return (mu + sigma * (double)X2);
        	return (degrees_to_radian(mu + sigma * (double)X2));
        }

        do
        {
                U1 = -1 + ((double)rand() / RAND_MAX) * 2;
                U2 = -1 + ((double)rand() / RAND_MAX) * 2;
                W = pow(U1, 2) + pow(U2, 2);
        } while (W >= 1 || W == 0);

        mult = sqrt((-2 * log(W)) / W);
        X1 = U1 * mult;
        X2 = U2 * mult;

//        call = !call;

//	return (mu + sigma * (double)X1);
        return (degrees_to_radian(mu + sigma * (double)X1));
}
