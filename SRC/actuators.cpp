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
int num_actuator_names = 2; // number of strings below and in enum
const char *actuator_names[] = { 
                                        "IDEAL_TWO_WHEEL", 
                                        "TWO_WHEEL"
                                        };
enum actuator_types {IDEAL_TWO_WHEEL = 0, TWO_WHEEL = 1,  NO_SENSOR};

/*-------------------------------------------------------------------------
 * (function: run_actuator)
 *-----------------------------------------------------------------------*/
void run_actuator(
		actuator_t *actuator,
		agent_t *agent,
		act_inputs_t *inputs,
		double current_time
	)
{
	/* call the control function of the current simulated robot */
	(*(actuator->fptr_actuator))(actuator, agent, inputs, current_time);	
}

/*-------------------------------------------------------------------------
 * (function: setup_function_for_actuator)
 *-----------------------------------------------------------------------*/
void setup_function_for_actuator(actuator_t *actuator, char *function_name)
{
	int actuator_function_id;
	
        actuator_function_id = return_string_in_list(function_name, (char**)actuator_names, num_actuator_names);

        switch(actuator_function_id)
        {
                /* PERMUTATION ENCODINGS */
                case IDEAL_TWO_WHEEL:
                        actuator->fptr_actuator = actuator_function_IDEAL_TWO_WHEEL;
			break;
                case TWO_WHEEL:
                        actuator->fptr_actuator = actuator_function_TWO_WHEEL;
			break;
		default:
			printf("EXIT - Agent with no actuator algorithm\n");
			exit(-1);
			break;
	}
}



