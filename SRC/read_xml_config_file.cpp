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
#include <stdarg.h>
#include "types.h"
#include "globals.h"

#include "read_xml_config_file.h"
#include "utils.h"
#include "robot_control.h"
#include "sensors.h"
#include "actuators.h"

// libxml includes
#include <libxml/xmlmemory.h> //#include <libxml/xmlmemory.h>
#include <libxml/parser.h> //#include <libxml/parser.h>

void read_xml_object(objects_t *object, xmlNodePtr shape_xmlptr, xmlDocPtr doc);

/*-------------------------------------------------------------------------
 * (function: read_config_file)
 * This reads an XML config file that specifies what we will do in the tool.
 *
 * See types.h to see the data structures used in this read.
 *-----------------------------------------------------------------------*/
void read_config_file(char *config_file_name)
{
	xmlDocPtr doc;
	xmlNodePtr top_xmlptr;
	int i;

	doc = xmlParseFile(config_file_name);
	
	if (doc == NULL ) 
	{
		fprintf(stderr,"Document not parsed successfully. \n");
		return;
	}
	
	top_xmlptr = xmlDocGetRootElement(doc);
	
	if (top_xmlptr == NULL) 
	{
		fprintf(stderr,"empty document\n");
		xmlFreeDoc(doc);
		return;
	}
	
	if (xmlStrcmp(top_xmlptr->name, (const xmlChar *) "centurion_config")) 
	{
		fprintf(stderr,"document of the wrong type, root node != centurion_config");
		xmlFreeDoc(doc);
		return;
	}
	
	top_xmlptr = top_xmlptr->xmlChildrenNode;
	while (top_xmlptr != NULL) 
	{
		xmlChar *string_data;

		if ((!xmlStrcmp(top_xmlptr->name, (const xmlChar *)"system")))
		{
			xmlNodePtr system_params_xmlptr = top_xmlptr->xmlChildrenNode;
			while (system_params_xmlptr != NULL) 
			{
				if ((!xmlStrcmp(system_params_xmlptr->name, (const xmlChar *)"rand_seed")))
				{
					string_data = xmlNodeListGetString(doc, system_params_xmlptr->xmlChildrenNode, 1);
					sim_system.rand_seed = atoi((char*)string_data);
					xmlFree(string_data);
				}
				else if ((!xmlStrcmp(system_params_xmlptr->name, (const xmlChar *)"simulation_type")))
				{
					string_data = xmlNodeListGetString(doc, system_params_xmlptr->xmlChildrenNode, 1);
					sim_system.simulation_type = (char*)string_data;
				}
				else if ((!xmlStrcmp(system_params_xmlptr->name, (const xmlChar *)"debug_file_out")))
				{
					string_data = xmlNodeListGetString(doc, system_params_xmlptr->xmlChildrenNode, 1);
					sim_system.debug_file_out = (char*)string_data;
				}
				else if ((!xmlStrcmp(system_params_xmlptr->name, (const xmlChar *)"sim_log_file_out")))
				{
					string_data = xmlNodeListGetString(doc, system_params_xmlptr->xmlChildrenNode, 1);
					sim_system.sim_log_file_out = (char*)string_data;
				}

				system_params_xmlptr = system_params_xmlptr->next;
			}
		}
		else if ((!xmlStrcmp(top_xmlptr->name, (const xmlChar *)"environment")))
		{
			xmlNodePtr environment_params_xmlptr = top_xmlptr->xmlChildrenNode;
			while (environment_params_xmlptr != NULL) 
			{
				if ((!xmlStrcmp(environment_params_xmlptr->name, (const xmlChar *)"real_size_x_in_m")))
				{
					string_data = xmlNodeListGetString(doc, environment_params_xmlptr->xmlChildrenNode, 1);
					environment.real_size_x_in_m = atof((char*)string_data);
					xmlFree(string_data);
				}
				else if ((!xmlStrcmp(environment_params_xmlptr->name, (const xmlChar *)"real_size_y_in_m")))
				{
					string_data = xmlNodeListGetString(doc, environment_params_xmlptr->xmlChildrenNode, 1);
					environment.real_size_y_in_m = atof((char*)string_data);
					xmlFree(string_data);
				}
				else if ((!xmlStrcmp(environment_params_xmlptr->name, (const xmlChar *)"sim_time_computation_epoch_s")))
				{
					string_data = xmlNodeListGetString(doc, environment_params_xmlptr->xmlChildrenNode, 1);
					environment.sim_time_computation_epoch_s = atof((char*)string_data);
					xmlFree(string_data);
				}
				else if ((!xmlStrcmp(environment_params_xmlptr->name, (const xmlChar *)"sim_time_s")))
				{
					string_data = xmlNodeListGetString(doc, environment_params_xmlptr->xmlChildrenNode, 1);
					environment.sim_time_s = atof((char*)string_data);
					xmlFree(string_data);
				}
				else if ((!xmlStrcmp(environment_params_xmlptr->name, (const xmlChar *)"boundary_walls")))
				{
					string_data = xmlNodeListGetString(doc, environment_params_xmlptr->xmlChildrenNode, 1);
					if (strcmp((char*)string_data, "FALSE") == 0)
					{
						environment.boundary_walls = FALSE;
					}
					else
					{
						environment.boundary_walls = TRUE;
					}
					xmlFree(string_data);
				}
				else if ((!xmlStrcmp(environment_params_xmlptr->name, (const xmlChar *)"objects")))
				{
					xmlNodePtr objects_xmlptr = environment_params_xmlptr->xmlChildrenNode;
					int objects_idx;
					while (objects_xmlptr != NULL) 
					{
						if ((!xmlStrcmp(objects_xmlptr->name, (const xmlChar *)"num_objects")))
						{
							string_data = xmlNodeListGetString(doc, objects_xmlptr->xmlChildrenNode, 1);
							environment.num_objects = atoi((char*)string_data);
							xmlFree(string_data);
							/* allocate the object data structures */
							environment.objects = (objects_t**)malloc(sizeof(objects_t*)*environment.num_objects);
							for (i = 0; i < environment.num_objects; i++)
							{
								environment.objects[i] = (objects_t*)malloc(sizeof(objects_t));
							}

							objects_idx = 0;
						}
						else if ((!xmlStrcmp(objects_xmlptr->name, (const xmlChar *)"object")))
						{
							read_xml_object(environment.objects[objects_idx], objects_xmlptr->xmlChildrenNode, doc);
							objects_idx ++;
						}

						objects_xmlptr = objects_xmlptr->next;
					}
					oassert(objects_idx == environment.num_objects) 
				}

				environment_params_xmlptr = environment_params_xmlptr->next;
			}
		}
		else if ((!xmlStrcmp(top_xmlptr->name, (const xmlChar *)"agents")))
		{
			xmlNodePtr agent_groups_xmlptr = top_xmlptr->xmlChildrenNode;
			int agent_group_idx = 0;

			while (agent_groups_xmlptr != NULL) 
			{
				if ((!xmlStrcmp(agent_groups_xmlptr->name, (const xmlChar *)"num_agent_groups")))
				{
					string_data = xmlNodeListGetString(doc, agent_groups_xmlptr->xmlChildrenNode, 1);
					agent_groups.num_agent_groups = atoi((char*)string_data);
					xmlFree(string_data);
					/* allocate the groups */
					agent_groups.agent_group = (agent_group_t**)malloc(sizeof(agent_group_t*)*agent_groups.num_agent_groups);
					for (i = 0; i < agent_groups.num_agent_groups; i++)
					{
						agent_groups.agent_group[i] = (agent_group_t*)malloc(sizeof(agent_group_t)*agent_groups.num_agent_groups);
					}
					agent_group_idx = 0;
				}
				else if ((!xmlStrcmp(agent_groups_xmlptr->name, (const xmlChar *)"agent_group")))
				{
					xmlNodePtr agent_group_xmlptr = agent_groups_xmlptr->xmlChildrenNode;

					while (agent_group_xmlptr != NULL) 
					{
						if ((!xmlStrcmp(agent_group_xmlptr->name, (const xmlChar *)"num_agents")))
						{
							string_data = xmlNodeListGetString(doc, agent_group_xmlptr->xmlChildrenNode, 1);
							agent_groups.agent_group[agent_group_idx]->num_agents = atoi((char*)string_data);
							xmlFree(string_data);
							/* allocate the agents */
							agent_groups.agent_group[agent_group_idx]->agents = (agent_t**)malloc(sizeof(agent_t*)*agent_groups.agent_group[agent_group_idx]->num_agents);

							for (i = 0; i < agent_groups.agent_group[agent_group_idx]->num_agents; i++)
							{
								agent_groups.agent_group[agent_group_idx]->agents[i] = (agent_t*)malloc(sizeof(agent_t));
								/* setup the back pointer so we can get from an individual to it's groups data */
								agent_groups.agent_group[agent_group_idx]->agents[i]->agent_group = agent_groups.agent_group[agent_group_idx];
								/* all agents start in state 0 */
								agent_groups.agent_group[agent_group_idx]->agents[i]->CURRENT_STATE = 0;
							}
						}
						else if ((!xmlStrcmp(agent_group_xmlptr->name, (const xmlChar *)"initialization_of_agents")))
						{
							xmlNodePtr initialization_xmlptr = agent_group_xmlptr->xmlChildrenNode;
							int agent_idx;

							while (initialization_xmlptr != NULL)
							{
								if ((!xmlStrcmp(initialization_xmlptr->name, (const xmlChar *)"list")))
								{
									xmlNodePtr list_xmlptr = initialization_xmlptr->xmlChildrenNode;
	
									agent_idx = 0;
									while (list_xmlptr != NULL)
									{
										if ((!xmlStrcmp(list_xmlptr->name, (const xmlChar *)"x")))
				                                                {
				                                                        string_data = xmlNodeListGetString(doc, list_xmlptr->xmlChildrenNode, 1);
				                                                        agent_groups.agent_group[agent_group_idx]->agents[agent_idx]->x = atof((char*)string_data);
				                                                        xmlFree(string_data);
										}
										else if ((!xmlStrcmp(list_xmlptr->name, (const xmlChar *)"y")))
				                                                {
				                                                        string_data = xmlNodeListGetString(doc, list_xmlptr->xmlChildrenNode, 1);
				                                                        agent_groups.agent_group[agent_group_idx]->agents[agent_idx]->y = atof((char*)string_data);
				                                                        xmlFree(string_data);
										}
										else if ((!xmlStrcmp(list_xmlptr->name, (const xmlChar *)"angle")))
				                                                {
				                                                        string_data = xmlNodeListGetString(doc, list_xmlptr->xmlChildrenNode, 1);
				                                                        agent_groups.agent_group[agent_group_idx]->agents[agent_idx]->angle = atof((char*)string_data);
				                                                        xmlFree(string_data);
	
											agent_idx ++;
										}
										list_xmlptr = list_xmlptr->next;
									}

									oassert (agent_idx == agent_groups.agent_group[agent_group_idx]->num_agents);
								}
								else if ((!xmlStrcmp(initialization_xmlptr->name, (const xmlChar *)"all_agent_function")))
								{
									string_data = xmlNodeListGetString(doc, initialization_xmlptr->xmlChildrenNode, 1);
									agent_groups.agent_group[agent_group_idx]->initialization_function = (char*)string_data;
									/* point where an initialization function would be called */
									oassert(FALSE);

									agent_idx ++;
								}

								initialization_xmlptr = initialization_xmlptr->next;
							}
			                        }
						else if ((!xmlStrcmp(agent_group_xmlptr->name, (const xmlChar *)"object")))
						{
							agent_groups.agent_group[agent_group_idx]->shape = (objects_t*)malloc(sizeof(objects_t));	
							read_xml_object(agent_groups.agent_group[agent_group_idx]->shape, agent_group_xmlptr->xmlChildrenNode, doc);
						}
						else if ((!xmlStrcmp(agent_group_xmlptr->name, (const xmlChar *)"sensors")))
						{
							xmlNodePtr sensors_xmlptr = agent_group_xmlptr->xmlChildrenNode;
							int sensor_idx = 0;
							while (sensors_xmlptr != NULL)
							{
								if ((!xmlStrcmp(sensors_xmlptr->name, (const xmlChar *)"num_sensors")))
		                                                {
		                                                        string_data = xmlNodeListGetString(doc, sensors_xmlptr->xmlChildrenNode, 1);
		                                                        agent_groups.agent_group[agent_group_idx]->num_sensors = atoi((char*)string_data);
		                                                        xmlFree(string_data);
									/* allocate sensors */
		                                                        agent_groups.agent_group[agent_group_idx]->sensors = (sensor_t**)malloc(sizeof(sensor_t*)*agent_groups.agent_group[agent_group_idx]->num_sensors);
									for (i = 0; i < agent_groups.agent_group[agent_group_idx]->num_sensors; i++)
									{
										agent_groups.agent_group[agent_group_idx]->sensors[i] = (sensor_t*)malloc(sizeof(sensor_t));
										/* initialize sensor */
										agent_groups.agent_group[agent_group_idx]->sensors[i]->time_of_last_read_s = 0;
									}
									sensor_idx = 0;

								}
								else if ((!xmlStrcmp(sensors_xmlptr->name, (const xmlChar *)"sensor")))
								{
									xmlNodePtr sensor_xmlptr = sensors_xmlptr->xmlChildrenNode;

		                                                        while (sensor_xmlptr != NULL)
		                                                        {
		                                                                if ((!xmlStrcmp(sensor_xmlptr->name, (const xmlChar *)"type")))
		                                                                {
		                                                                        string_data = xmlNodeListGetString(doc, sensor_xmlptr->xmlChildrenNode, 1);

											/* setup function call */
											setup_function_for_sensor(agent_groups.agent_group[agent_group_idx]->sensors[sensor_idx], (char*)string_data);

		                                                                        xmlFree(string_data);
		                                                                }
		                                                                else if ((!xmlStrcmp(sensor_xmlptr->name, (const xmlChar *)"direction_on_agent")))
		                                                                {
		                                                                        string_data = xmlNodeListGetString(doc, sensor_xmlptr->xmlChildrenNode, 1);
		                                                                       	agent_groups.agent_group[agent_group_idx]->sensors[sensor_idx]->angle = atof((char*)string_data);
		                                                                        xmlFree(string_data);
										}
										else if ((!xmlStrcmp(sensor_xmlptr->name, (const xmlChar *)"sim_time_computation_epoch_s")))
		                                                                {
		                                                                        string_data = xmlNodeListGetString(doc, sensor_xmlptr->xmlChildrenNode, 1);
		                                                                       	agent_groups.agent_group[agent_group_idx]->sensors[sensor_idx]->sim_time_computation_epoch_s = atof((char*)string_data);
		                                                                        xmlFree(string_data);
											sensor_idx ++;
										}
		
		                                                                sensor_xmlptr = sensor_xmlptr->next;
		                                                        }
								}
							
								sensors_xmlptr = sensors_xmlptr->next;
							}
							oassert(sensor_idx == agent_groups.agent_group[agent_group_idx]->num_sensors);
			                        }
						else if ((!xmlStrcmp(agent_group_xmlptr->name, (const xmlChar *)"actuators")))
						{
							xmlNodePtr actuators_xmlptr = agent_group_xmlptr->xmlChildrenNode;
							int actuator_idx = 0;
							while (actuators_xmlptr != NULL)
							{
								if ((!xmlStrcmp(actuators_xmlptr->name, (const xmlChar *)"num_actuators")))
		                                                {
		                                                        string_data = xmlNodeListGetString(doc, actuators_xmlptr->xmlChildrenNode, 1);
		                                                        agent_groups.agent_group[agent_group_idx]->num_actuators = atoi((char*)string_data);
		                                                        xmlFree(string_data);
									/* allocate actuators */
		                                                        agent_groups.agent_group[agent_group_idx]->actuators = (actuator_t**)malloc(sizeof(actuator_t*)*agent_groups.agent_group[agent_group_idx]->num_actuators);
									for (i = 0; i < agent_groups.agent_group[agent_group_idx]->num_actuators; i++)
									{
										agent_groups.agent_group[agent_group_idx]->actuators[i] = (actuator_t*)malloc(sizeof(actuator_t));
										agent_groups.agent_group[agent_group_idx]->actuators[i]->general_memory = NULL; 
										agent_groups.agent_group[agent_group_idx]->actuators[i]->initialized = FALSE;
									}
									actuator_idx = 0;

								}
								else if ((!xmlStrcmp(actuators_xmlptr->name, (const xmlChar *)"actuator")))
								{
									xmlNodePtr actuator_xmlptr = actuators_xmlptr->xmlChildrenNode;

		                                                        while (actuator_xmlptr != NULL)
		                                                        {
		                                                                if ((!xmlStrcmp(actuator_xmlptr->name, (const xmlChar *)"type")))
		                                                                {
		                                                                        string_data = xmlNodeListGetString(doc, actuator_xmlptr->xmlChildrenNode, 1);
											/* setup actuator function */
											setup_function_for_actuator(agent_groups.agent_group[agent_group_idx]->actuators[actuator_idx], (char*)string_data);

											actuator_idx++;
		                                                                }
		
		                                                                actuator_xmlptr = actuator_xmlptr->next;
		                                                        }
								}
								actuators_xmlptr = actuators_xmlptr->next;
							}
							oassert(actuator_idx == agent_groups.agent_group[agent_group_idx]->num_actuators);
			                        }
						else if ((!xmlStrcmp(agent_group_xmlptr->name, (const xmlChar *)"control")))
						{
							xmlNodePtr control_xmlptr = agent_group_xmlptr->xmlChildrenNode;

							while (control_xmlptr != NULL)
							{
								if ((!xmlStrcmp(control_xmlptr->name, (const xmlChar *)"control_algorithm")))
		                                                {
		                                                        string_data = xmlNodeListGetString(doc, control_xmlptr->xmlChildrenNode, 1);
									/* hookup the control function */
									setup_function_for_control(agent_groups.agent_group[agent_group_idx], (char*)string_data);
									xmlFree(string_data);
								}
								control_xmlptr = control_xmlptr->next;
							}
							/* NOTE assumes control is last item and always there */
							agent_group_idx ++;
			                        }

						agent_group_xmlptr = agent_group_xmlptr->next;
					}
				}

				agent_groups_xmlptr = agent_groups_xmlptr->next;
			}
			oassert(agent_group_idx == agent_groups.num_agent_groups)
		}
		else if ((!xmlStrcmp(top_xmlptr->name, (const xmlChar *)"atons")))
		{
			xmlNodePtr aton_xmlptr = top_xmlptr->xmlChildrenNode;
			while (aton_xmlptr != NULL) 
			{
				aton_xmlptr = aton_xmlptr->next;
			}
		}

		top_xmlptr = top_xmlptr->next;
	}
	
	xmlFreeDoc(doc);
	return;
}

/*-------------------------------------------------------------------------
 * (function: read_xml_object)
 *-----------------------------------------------------------------------*/
void read_xml_object(objects_t *object, xmlNodePtr shape_xmlptr, xmlDocPtr doc)
{
	xmlChar *string_data;

	object->radius = 0.0;
	object->x1 = 0.0;
	object->y1 = 0.0;
	object->x2 = 0.0;
	object->y2 = 0.0;
	object->x3 = 0.0;
	object->y3 = 0.0;
	object->x4 = 0.0;
	object->y4 = 0.0;

	if ((!xmlStrcmp(shape_xmlptr->name, (const xmlChar *)"sphere")))
	{
		xmlNodePtr shape_details_xmlptr = shape_xmlptr->xmlChildrenNode;
		while (shape_details_xmlptr != NULL)
		{
			object->type = SPHERE;
			if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"x")))
                       	{
                       		string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
                               	object->x1 = atof((char*)string_data);
                               	xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"y")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->y1 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"radius")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->radius = atof((char*)string_data);
				xmlFree(string_data);
			}
			shape_details_xmlptr = shape_details_xmlptr->next;
		}
	}
	else if ((!xmlStrcmp(shape_xmlptr->name, (const xmlChar *)"quadrilateral")))
	{
		xmlNodePtr shape_details_xmlptr = shape_xmlptr->xmlChildrenNode;
		while (shape_details_xmlptr != NULL)
		{
			object->type = QUADRILATERAL;
			if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"x")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->x1 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"y")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->y1 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"x1")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->x2 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"y1")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->y2 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"x2")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->x3 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"y2")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->y3 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"x3")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->x4 = atof((char*)string_data);
				xmlFree(string_data);
			}
			else if ((!xmlStrcmp(shape_details_xmlptr->name, (const xmlChar *)"y3")))
			{
				string_data = xmlNodeListGetString(doc, shape_details_xmlptr->xmlChildrenNode, 1);
				object->y4 = atof((char*)string_data);
				xmlFree(string_data);
			}
			shape_details_xmlptr = shape_details_xmlptr->next;
		}
	}
}

/*-------------------------------------------------------------------------
 * (function: free_configuartion)
 *-----------------------------------------------------------------------*/
void free_xml_data(char *config_file_name)
{
}
