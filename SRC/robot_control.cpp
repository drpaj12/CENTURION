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
int num_control_algorithm_names = 5;
const char *control_algorithm_name[] = { 
                                        "OVERLORD", 
                                        "BASIC_AVOID_ICRA",
					"BASIC_AVOID_ICRA_W_BAYESIAN",
					"BOIDS",
				        "SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE"	
                                        };

enum control_algorithm_type {OVERLORD = 0, BASIC_AVOID_ICRA = 1, BASIC_AVOID_ICRA_W_BAYESIAN = 2, BOIDS = 3, SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE, NO_CONTROL};

/*-------------------------------------------------------------------------
 * (function: run_agent_control)
 *-----------------------------------------------------------------------*/
void run_agent_control(
		agent_t *agent,
		double current_time
	) 
{
	/* call the control function of the current simulated robot */
	(*(agent->agent_group->fptr_control_algorithm))(agent, current_time);	
}

/*-------------------------------------------------------------------------
 * (function: setup_function_for_control)
 *-----------------------------------------------------------------------*/
void setup_function_for_control(agent_group_t *agent_group, char *function_name)
{
	int control_function_id;
	
	oassert(NO_CONTROL == num_control_algorithm_names);

        control_function_id = return_string_in_list(function_name, (char**)control_algorithm_name, num_control_algorithm_names);

        switch(control_function_id)
        {
                /* PERMUTATION ENCODINGS */
                case OVERLORD:
                        agent_group->fptr_control_algorithm = control_algorithm_OVERLORD;
			break;
                case BASIC_AVOID_ICRA:
                        agent_group->fptr_control_algorithm = control_algorithm_BASIC_AVOID_ICRA;
			break;
                case BASIC_AVOID_ICRA_W_BAYESIAN:
                        agent_group->fptr_control_algorithm = control_algorithm_BASIC_AVOID_ICRA_W_BAYESIAN;
			break;
                case BOIDS:
                        agent_group->fptr_control_algorithm = NULL;
			oassert(FALSE);
			break;
                case SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE:
                        agent_group->fptr_control_algorithm = control_algorithm_SIMPLE_MOVE_IN_SQUARE_AND_STOP_W_OBSTACLE;
			break;
		default:
			printf("EXIT - Agent with no control algorithm\n");
			exit(-1);
			break;
	}
}
