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
#include "robot_control.h"
#include "log_file_xml.h"

/* globals */
sim_obj_t **sim_objects;
int num_sim_objects;

/*-------------------------------------------------------------------------
 * (function: setup_simulation)
 *-----------------------------------------------------------------------*/
void setup_simulation() 
{
	int i,j;
	int sim_object_idx;

	num_sim_objects = 0;

	/* Init agents and objects onto sim objects */
	num_sim_objects += environment.num_objects;
	for (i = 0; i < agent_groups.num_agent_groups; i++)
	{
		num_sim_objects += agent_groups.agent_group[i]->num_agents;
	}

	/* allocate objects */
	sim_objects = (sim_obj_t **)malloc(sizeof(sim_obj_t*) * num_sim_objects);
	for (i = 0; i < num_sim_objects; i++)
	{
		sim_objects[i] = (sim_obj_t *)malloc(sizeof(sim_obj_t));
		sim_objects[i]->object = NULL;
		sim_objects[i]->agent = NULL;
	}

	/* load objects to sim_objects */
	sim_object_idx = 0;

	for (i = 0; i < environment.num_objects; i++)
	{
		sim_objects[i]->type = OBJECT;
		sim_objects[i]->object = environment.objects[i];
	}

	sim_object_idx = environment.num_objects;

	for (i = 0; i < agent_groups.num_agent_groups; i++)
	{
		for (j = 0; j < agent_groups.agent_group[i]->num_agents; j++)
		{
			sim_objects[sim_object_idx]->type = AGENT;
			sim_objects[sim_object_idx]->agent = agent_groups.agent_group[i]->agents[j];

			sim_object_idx ++;
		}
	}
}
/*-------------------------------------------------------------------------
 * (function: simulation_loop)
 *-----------------------------------------------------------------------*/
void simulation_loop() 
{
	int i;
	double current_time = 0;
	short exit = FALSE;

	while (exit == FALSE)
	{
		/* update time */
		current_time += environment.sim_time_computation_epoch_s;

		/* do processing of objects */
		for (i = 0; i < num_sim_objects; i++)
		{
			if (sim_objects[i]->type == OBJECT)
			{
				continue;
			}
			else if (sim_objects[i]->type == AGENT)
			{
				run_agent_control(sim_objects[i]->agent, current_time);
			}
		}

		/* check for crashes */

		/* update world */
		sim_system.output_log_tab_step = output_log_file_xml_time_step_start(sim_system.output_log_tab_step, current_time);
		for (i = 0; i < num_sim_objects; i++)
		{
			if (sim_objects[i]->type == OBJECT)
			{
				continue;
			}
			else if (sim_objects[i]->type == AGENT && sim_objects[i]->agent->not_physical_agent == FALSE)
			{
				sim_system.output_log_tab_step = output_log_file_xml_time_step_agent(sim_system.output_log_tab_step, i, sim_objects[i]->agent->circle->center.x, sim_objects[i]->agent->circle->center.y, sim_objects[i]->agent->angle);
//				fprintf(sim_system.Fsim_log_out, "time:%f - %d - x:%f, y:%f, angle:%f, angle_d:%f\n", current_time, i, sim_objects[i]->agent->circle->center.x, sim_objects[i]->agent->circle->center.y, sim_objects[i]->agent->angle, sim_objects[i]->agent->angle * (180.0 / PI));
			}
		}
		sim_system.output_log_tab_step = output_log_file_xml_time_step_stop(sim_system.output_log_tab_step);

		/* check for exit */
		if (environment.sim_time_s < current_time)
		{
			printf("Simulation done at time: %f\n", current_time);
			exit = TRUE;
		}
	}
}
	
