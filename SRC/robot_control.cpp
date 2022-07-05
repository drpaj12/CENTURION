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

#include "OVERLORD_control_algorithm.h"
#include "BASIC_AVOID_ICRA_control_algorithm.h"

/* globals */
const char *control_algorithm_name[] = { 
                                        "OVERLORD", 
                                        "BASIC_AVOID_ICRA"/*,
                                        "BOIDS",*/ 
                                        };

enum control_algorithm_type {OVERLORD = 0, BASIC_AVOID_ICRA = 1 /*, BOIDS*/, NO_CONTROL};

/*-------------------------------------------------------------------------
 * (function: run_agent_control)
 *-----------------------------------------------------------------------*/
void run_agent_control(
		void (*fptr_robot_control)(agent_t *agent, double current_time),
		agent_t *agent,
		double current_time
	) 
{
	/* call the control function of the current simulated robot */
	(*fptr_robot_control)(agent, current_time);	
}

/*-------------------------------------------------------------------------
 * (function: run_agent_control)
 *-----------------------------------------------------------------------*/
void setup_function_for_control(agent_group_t *agent_group, char *function_name)
{
	int control_function_id;
	control_algorithm_type control_function_d;
	
        control_function_id = return_string_in_list(function_name, (char**)control_algorithm_name, 2);

        switch(control_function_id)
        {
                /* PERMUTATION ENCODINGS */
                case OVERLORD:
                        agent_group->control_algorithm = OVERLORD_control_algorithm;
			break;
                case BASIC_AVOID_ICRA:
                        agent_group->control_algorithm = BASIC_AVOID_ICRA_control_algorithm;
			break;
		default:
                        agent_group->control_algorithm = NULL;
			oassert(FALSE);
			break;
	}
}



