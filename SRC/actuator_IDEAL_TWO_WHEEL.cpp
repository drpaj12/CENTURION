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

/* globals */

/*-------------------------------------------------------------------------
 * (function: )
 *-----------------------------------------------------------------------*/
void actuator_function_IDEAL_TWO_WHEEL(actuator_t *actuator, agent_t *agent, act_inputs_t *inputs, double current_time) 
{
	printf("IDEAL_TWO WHEEL ACTUATOR called\n");

	if (inputs->left == 1 && inputs->right == 1)
		printf("moving forward\n");
	else if (inputs->left == 0 && inputs->right == 0)
		printf("stopped\n");
	else if (inputs->left == 1 && inputs->right == 0)
		printf("turning left\n");
	else if (inputs->left == 0 && inputs->right == 1)
		printf("turning right\n");
	else 
		printf("moving confused\n");
}

