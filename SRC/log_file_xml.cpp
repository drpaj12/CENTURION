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
 * (function: tabs_to_line)
 *-----------------------------------------------------------------------*/
void tabs_to_line(int tabs) 
{
	int i;

	for (i = 0; i < tabs; i++)
	{
		fprintf(sim_system.Fsim_log_out, "    ");
	}
}

/*-------------------------------------------------------------------------
 * (function: output_log_file_xml_header)
 *-----------------------------------------------------------------------*/
int output_log_file_xml_header() 
{
	int num_tabs = 0;
	int i;

	fprintf(sim_system.Fsim_log_out, "<data_log>\n");
	num_tabs ++;	
	
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<time_step_in_s>%f</time_step_in_s>\n", environment.sim_time_computation_epoch_s);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<sim_x_in_m>%f</sim_x_in_m>\n", environment.real_size_x_in_m);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<sim_y_in_m>%f</sim_y_in_m>\n", environment.real_size_y_in_m);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<agent_radius>%f</agent_radius>\n", agent_groups.agent_group[1]->shape->circle->radius);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<object>\n");
	num_tabs ++;

	for (i = 0; i < environment.num_objects; i++)
	{
		if (environment.objects[i]->type == CIRCLE)
		{
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<circle>\n");
			num_tabs++;
			
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<x>%f</x>\n", environment.objects[i]->circle->center.x);
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<y>%f</y>\n", environment.objects[i]->circle->center.y);
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<radius>%f</radius>\n", environment.objects[i]->circle->radius);

			num_tabs--;
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "</circle>\n");
		}
		else if (environment.objects[i]->type == RECTANGLE)
		{
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<rectangle>\n");
			num_tabs++;
			
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<x>%f</x>\n", environment.objects[i]->rectangle->center.x);
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<y>%f</y>\n", environment.objects[i]->rectangle->center.y);
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<halfx>%f</halfx>\n", environment.objects[i]->rectangle->halfExtend.x);
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<halfy>%f</halfy>\n", environment.objects[i]->rectangle->halfExtend.y);
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "<rotation>%f</rotation>\n", environment.objects[i]->rectangle->rotation);

			num_tabs--;
			tabs_to_line(num_tabs);
			fprintf(sim_system.Fsim_log_out, "</rectangle>\n");
		}
	}

	num_tabs --;
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "</object>\n");

	return num_tabs;
}
	
/*-------------------------------------------------------------------------
 * (function: output_log_file_xml_footer)
 *-----------------------------------------------------------------------*/
void output_log_file_xml_footer(int tabs) 
{
	int num_tabs = tabs;

	num_tabs--;
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "</data_log>\n");

	oassert(num_tabs == 0);
}
/*-------------------------------------------------------------------------
 * (function: output_log_file_xml_time_step_start)
 *-----------------------------------------------------------------------*/
int output_log_file_xml_time_step_start(int tabs, double current_time) 
{
	int num_tabs = tabs;

	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<time_step>\n");
	num_tabs++;

	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<time_at>%f</time_at>\n", current_time);

	return num_tabs;
}

/*-------------------------------------------------------------------------
 * (function: output_log_file_xml_time_step_start)
 *-----------------------------------------------------------------------*/
int output_log_file_xml_time_step_stop(int tabs) 
{
	int num_tabs = tabs;

	num_tabs--;
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "</time_step>\n");

	return num_tabs;
}

/*-------------------------------------------------------------------------
 * (function: output_log_file_xml_time_step_agent)
 *-----------------------------------------------------------------------*/
int output_log_file_xml_time_step_agent(int tabs, int agent_id, double x, double y, double angle) 
{
	int num_tabs = tabs;

	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<agent>\n");
	num_tabs++;

	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<agent_id>%d</agent_id>\n", agent_id);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<x>%f</x>\n", x);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<y>%f</y>\n", y);
	tabs_to_line(num_tabs);
	/* output in degrees */
	fprintf(sim_system.Fsim_log_out, "<angle>%f</angle>\n", angle*(180/PI));

	num_tabs--;
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "</agent>\n");

	return num_tabs;
}

/*-------------------------------------------------------------------------
 * (function: output_log_file_xml_time_step_sensor_beam_hit)
 *-----------------------------------------------------------------------*/
int output_log_file_xml_time_step_sensor_beam_hit(int tabs, line_segment_t *sensor_beam, vector_2D_t *point_intersect) 
{
	int num_tabs = tabs;

	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<sensor_beam>\n");
	num_tabs++;

	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<beam_x1>%f</beam_x1><beam_y1>%f</beam_y1>\n", sensor_beam->point1.x, sensor_beam->point1.y);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<beam_x2>%f</beam_x2><beam_y2>%f</beam_y2>\n", sensor_beam->point2.x, sensor_beam->point2.y);
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "<point_intersect_x>%f</point_intersect_x><point_intersect_y>%f</point_intersect_y>\n", point_intersect->x,  point_intersect->y);

	num_tabs--;
	tabs_to_line(num_tabs);
	fprintf(sim_system.Fsim_log_out, "</sensor_beam>\n");

	return num_tabs;
}
