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

/* globals */

/*-------------------------------------------------------------------------
 * (function: move_forward)
 *-----------------------------------------------------------------------*/
void move(agent_t *agent, double distance_in_m)
{
	agent->circle->center.x = agent->circle->center.x + cos(agent->angle) * distance_in_m;
	agent->circle->center.y = agent->circle->center.y + sin(agent->angle) * distance_in_m;
}

/*-------------------------------------------------------------------------
 * (function: move_with_drift)
 *-----------------------------------------------------------------------*/
void move_with_drift(agent_t *agent, double distance_in_m, double angle_offset)
{
	agent->circle->center.x = agent->circle->center.x + cos(agent->angle+angle_offset) * distance_in_m;
	agent->circle->center.y = agent->circle->center.y + sin(agent->angle+angle_offset) * distance_in_m;
}


/*-------------------------------------------------------------------------
 * (function: move_forward)
 *-----------------------------------------------------------------------*/
void turn(agent_t *agent, double angle_in_rad)
{
	agent->angle = agent->angle + angle_in_rad;

	/* keep angle between 0 and 360 degrees from a Radian point of view 0 to 2PI */
	if (agent->angle > twoPI)
	{
		agent->angle -= twoPI;
	}
	else if (agent->angle < 0)
	{
		agent->angle += twoPI;
	}
}
