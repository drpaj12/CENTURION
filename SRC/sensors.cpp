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
#include "collision_detection.h"
#include "log_file_xml.h"

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

/*-------------------------------------------------------------------------
 * (function: find_closest_object_on_beam_projection )
 *-----------------------------------------------------------------------*/
beam_sensor_t* find_closest_object_on_beam_projection(beam_sensor_t **sensor_reading, agent_t *agent_self, double x, double y, double beam_distance, double angle_radians)
{
	int i, j;
	points_t *points_of_intersect;

	vector_2D_t start_point;
	start_point.x = x;
	start_point.y = y;

	vector_2D_t end_point;
	end_point.x = x + cos(angle_radians) * beam_distance;
	end_point.y = y + sin(angle_radians) * beam_distance;

	line_segment_t beam_segment;
	beam_segment.point1 = start_point;
	beam_segment.point2 = end_point;

	circle_t *circle;
	oriented_rectangle_t *rectangle;

	//printf("sensor_beam line_segemnt -> x=%f y=%f to x1=%f y=%f\n", start_point.x, start_point.y, end_point.x, end_point.y);

	vector_2D_t point_of_intersect;
	sim_obj_t *closest_obj = NULL;
	double min_distance = 2*beam_distance;

	for (i = 0; i < num_sim_objects; i++)
	{
		circle = NULL;
		rectangle = NULL;

		sim_obj_t *potential_closest = sim_objects[i];
		double min_distance_pot = 0;

		if (sim_objects[i]->type == OBJECT)
		{
			if (sim_objects[i]->object->type == CIRCLE)
			{
				circle = sim_objects[i]->object->circle;
			}
			else if (sim_objects[i]->object->type == RECTANGLE)
			{
				rectangle = sim_objects[i]->object->rectangle;
			}
		}
		else if (sim_objects[i]->type == AGENT)
		{
			if (sim_objects[i]->agent == agent_self || sim_objects[i]->agent->not_physical_agent == TRUE)
				continue;
			else 
			{
				/* assume robots only spheres */
				oassert(sim_objects[i]->agent->agent_group->shape->type == CIRCLE);
				circle = sim_objects[i]->agent->circle;
			}
		}

		if (circle != NULL)
		{
			points_of_intersect = segment_intersects_circle_at(&beam_segment, circle); 

			if (points_of_intersect == NULL)
			{
				printf("NO BEAM HIT on CIRCLE (%f, %f, %f)\n", circle->center.x, circle->center.y, circle->radius);
			}
			else
			{

				printf("BEAM HIT CIRCLE(%f, %f, %f)\n", circle->center.x, circle->center.y, circle->radius);
				for (j = 0; j < points_of_intersect->num_points; j++)
				{
					printf("	POINT %d -> x=%f y=%f\n", j, points_of_intersect->points[j]->x, points_of_intersect->points[j]->y);
				}

			}
		}
		else if (rectangle != NULL)
		{
			points_of_intersect = segment_intersects_oriented_rectangle_at(&beam_segment, rectangle);

			vector_2D_t a;
			vector_2D_t b;
			vector_2D_t c;
			vector_2D_t d;
			oriented_rectangle_to_points(&a, &b, &c, &d, rectangle);

			if (points_of_intersect == NULL)
			{
				printf("NO BEAM HIT on RECTANGLE ((%f,%f), (%f,%f), (%f,%f), (%f,%f))\n", a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y);
			}
			else
			{
				printf("BEAM HIT on RECTANGLE ((%f,%f), (%f,%f), (%f,%f), (%f,%f))\n", a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y);
				for (j = 0; j < points_of_intersect->num_points; j++)
				{
					printf("	POINT %d -> x=%f y=%f\n", j, points_of_intersect->points[j]->x, points_of_intersect->points[j]->y);
				}
			}
		}

		if (points_of_intersect != NULL)
		{
			/* see if this point is closer than previous ones */
			if (points_of_intersect->num_points == 2)
			{
				double d0 = two_points_distance(points_of_intersect->points[0], &start_point);
				double d1 = two_points_distance(points_of_intersect->points[1], &start_point);
	
				if (d0 < min_distance && d0 < d1)
				{
					min_distance = d0;
					closest_obj = potential_closest;
					point_of_intersect.x = points_of_intersect->points[0]->x;
					point_of_intersect.y = points_of_intersect->points[0]->y;
				}
				else if (d1 < min_distance)
				{
					min_distance = d1;
					closest_obj = potential_closest;
					point_of_intersect.x = points_of_intersect->points[1]->x;
					point_of_intersect.y = points_of_intersect->points[1]->y;
				}
	
				free (points_of_intersect->points[0]);
				free (points_of_intersect->points[1]);
				free (points_of_intersect->points);
				free (points_of_intersect);
			}
			else if (points_of_intersect->num_points == 1)
			{
				double d0 = two_points_distance(points_of_intersect->points[0], &start_point);
	
				if (d0 < min_distance)
				{
					min_distance = d0;
					closest_obj = potential_closest;
					point_of_intersect.x = points_of_intersect->points[0]->x;
					point_of_intersect.y = points_of_intersect->points[0]->y;
				}
	
				free (points_of_intersect->points[0]);
				free (points_of_intersect->points);
				free (points_of_intersect);
			}
		}
	}

	if (closest_obj != NULL)
	{
		sensor_reading[0]->in_m = min_distance;
		sensor_reading[0]->angle_phi = 0.0;
		/* output sensor hit to log file */
		sim_system.output_log_tab_step = output_log_file_xml_time_step_sensor_beam_hit(sim_system.output_log_tab_step, &beam_segment, &point_of_intersect, min_distance);
	}
	else
	{
		sensor_reading[0]->in_m = -1;
		sensor_reading[0]->angle_phi = 0.0;
	}

}
