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
int num_sensor_names = 3; // number of strings below and in enum
const char *sensor_names[] = { 
                                        "IDEAL_BEAM", 
                                        "ULTRASONIC",
                                        "IR" 
                                        };
enum sensor_types {IDEAL_BEAM = 0, ULTRASONIC = 1 , IR = 2, NO_SENSOR};

/*-------------------------------------------------------------------------
 * (function: run_sensor)
 *-----------------------------------------------------------------------*/
void* run_sensor(
		sensor_t *sensor,
		agent_t *agent,
		double current_time
	) 
{
	/* call the control function of the current simulated robot */
	return (*(sensor->fptr_sensor))(sensor, agent, current_time);	
}

/*-------------------------------------------------------------------------
 * (function: setup_function_for_sensor)
 *-----------------------------------------------------------------------*/
void setup_function_for_sensor(sensor_t *sensor, char *function_name)
{
	int sensor_function_id;
	
        sensor_function_id = return_string_in_list(function_name, (char**)sensor_names, num_sensor_names);

        switch(sensor_function_id)
        {
                /* PERMUTATION ENCODINGS */
                case IDEAL_BEAM:
                        sensor->fptr_sensor = sensor_function_IDEAL_BEAM;
			break;
                case ULTRASONIC:
                        sensor->fptr_sensor = sensor_function_ULTRASONIC;
			break;
                case IR:
                        sensor->fptr_sensor = sensor_function_IR;
			break;
		default:
			printf("EXIT - Agent with no sensor algorithm\n");
			exit(-1);
			break;
	}
}



