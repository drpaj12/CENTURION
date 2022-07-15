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
#include "sensors.h"

/* globals */

#define BAYESIAN_STATE_SIZE  591
#define BAYESIAN_READS  4

typedef struct sensor_state_t_t sensor_state_t;
struct sensor_state_t_t
{
	double sense_completed_in_s;
	beam_sensor_t *sensor_reading;
	double *probability_array;
	short after_bayesian_reads;
};


double sum_array(double array[], int num_elements);
void normalize_array(double array[], int num_elements);
double make_bayesian_prediction(double *prob_array, double actual_distance, int restart) ;
double randnorm(double mu, double sigma);
double generate_characterized_sensor_read(double distance);
double generate_characterized_sensor_read_with_bayesian(double distance, double *probability_array, int num_reads);

/* GAUSSIAN code from
 * https://kcru.lawsonresearch.ca/research/srk/normalDBN_random.html
 */

/*-------------------------------------------------------------------------
 * (function: )
 *-----------------------------------------------------------------------*/
void* sensor_function_ULTRASONIC_W_BAYESIAN(sensor_t *sensor, agent_t *agent, double current_time) 
{
	beam_sensor_t *sensor_reading;
	sensor_state_t* sensor_state;
	double* probability_array;

	if (agent->sensor_memories[sensor->sensor_idx] == NULL)
	{	
		sensor_state = (sensor_state_t*)malloc(sizeof(sensor_state_t));
		sensor_state->sense_completed_in_s = 0;
		sensor_reading = (beam_sensor_t*)malloc(sizeof(beam_sensor_t));
		sensor_state->sensor_reading = sensor_reading;
		probability_array = (double*)malloc(sizeof(double)*571);
		sensor_state->probability_array = probability_array;
		sensor_state->after_bayesian_reads = 0;

		/* store as memory */
		agent->sensor_memories[sensor->sensor_idx] = (void*)sensor_state;
	}
	else
	{
		/* extract memory */
		sensor_state = (sensor_state_t*)(agent->sensor_memories[sensor->sensor_idx]);
		sensor_reading = sensor_state->sensor_reading;
		probability_array = sensor_state->probability_array;
	}

	if (sensor_state->sense_completed_in_s < current_time)
	{
		/* next sense completed at */
		sensor_state->sense_completed_in_s = current_time + sensor->sim_time_computation_epoch_s;
		if (sensor_state->after_bayesian_reads == BAYESIAN_READS)
			sensor_state->after_bayesian_reads = 0;
		else
			sensor_state->after_bayesian_reads ++;

		/* get current reading */
		//find_closest_object_on_beam_projection(&sensor_reading, agent, agent->circle->center.x, agent->circle->center.y, agent->circle->radius+.5, agent->angle);
		find_closest_object_on_beam_projection(&sensor_reading, agent, agent->circle->center.x + (cos(agent->angle)*agent->circle->radius) , agent->circle->center.y + (sin(agent->angle)*agent->circle->radius), .5, agent->angle);
		/* if we get a read use characterization */
		if (sensor_reading->in_m != -1)
		{
			printf("read:%d: Object is %f", sensor_state->after_bayesian_reads, sensor_reading->in_m);
			/* the function is written in cm hence the *100 and /100 */
			sensor_reading->in_m = generate_characterized_sensor_read_with_bayesian(100*sensor_reading->in_m, probability_array, sensor_state->after_bayesian_reads) / 100; 
			printf(" but sensor read is %f\n", sensor_reading->in_m);
		}

		sensor_reading->new_data = TRUE;
		sensor_reading->reads = sensor_state->after_bayesian_reads;
	}
	else
	{
		sensor_reading->new_data = FALSE;
	}

	return (void*)sensor_reading;
}


/*-------------------------------------------------------------------------
 * (function: )
 *-----------------------------------------------------------------------*/
void* sensor_function_ULTRASONIC(sensor_t *sensor, agent_t *agent, double current_time) 
{
	beam_sensor_t *sensor_reading;
	sensor_state_t* sensor_state;

	if (agent->sensor_memories[sensor->sensor_idx] == NULL)
	{	
		sensor_state = (sensor_state_t*)malloc(sizeof(sensor_state_t));
		sensor_state->sense_completed_in_s = 0;
		sensor_reading = (beam_sensor_t*)malloc(sizeof(beam_sensor_t));
		sensor_state->sensor_reading = sensor_reading;

		/* store as memory */
		agent->sensor_memories[sensor->sensor_idx] = (void*)sensor_state;
	}
	else
	{
		/* extract memory */
		sensor_state = (sensor_state_t*)(agent->sensor_memories[sensor->sensor_idx]);
		sensor_reading = sensor_state->sensor_reading;
	}

	if (sensor_state->sense_completed_in_s < current_time)
	{
		/* next sense completed at */
		sensor_state->sense_completed_in_s = current_time + sensor->sim_time_computation_epoch_s;

		/* get current reading */
		//find_closest_object_on_beam_projection(&sensor_reading, agent, agent->circle->center.x, agent->circle->center.y, agent->circle->radius+.5, agent->angle);
		find_closest_object_on_beam_projection(&sensor_reading, agent, agent->circle->center.x + (cos(agent->angle)*agent->circle->radius) , agent->circle->center.y + (sin(agent->angle)*agent->circle->radius), .5, agent->angle);
		/* if we get a read use characterization */
		if (sensor_reading->in_m != -1)
		{
			printf("Object is %f", sensor_reading->in_m);
			sensor_reading->in_m = generate_characterized_sensor_read(100*sensor_reading->in_m) / 100; 
			printf(" but sensor read is %f\n", sensor_reading->in_m);
		}

		sensor_reading->new_data = TRUE;
	}
	else
	{
		sensor_reading->new_data = FALSE;
	}

	return (void*)sensor_reading;
}

double sum_array(double array[], int num_elements)
{
	double sum = 0;
	for (int i = 0; i<num_elements; i++)
	{
		sum = sum + array[i];
	}
	return(sum);
}

void normalize_array(double array[], int num_elements)
{
	double sum = sum_array(array, num_elements);
	for (int i = 0; i<num_elements; i++)
	{
		array[i] = array[i] / sum;
	}
}

double make_bayesian_prediction(double *prob_array, double actual_distance, int restart) 
{
	int i;
	int at_idx = 0;
	double return_distance;
	double find_max = -1;
	// Variable to notify reading and 
	static int sampleCount = 0;
	static int init = 1;
	/* Storing finite number of state */
	double state[BAYESIAN_STATE_SIZE];
	double num = 1;
	for (int i = 0; i < BAYESIAN_STATE_SIZE; i++) {
		state[i] = num;
		num += 0.1;
	}
	/* Array to store prediction probability */
	double en = 1.0 / (double)BAYESIAN_STATE_SIZE;
	//static double prob_array[BAYESIAN_STATE_SIZE];
	// Not sure why iteration is here - paj commented out
	// RESTART after BAYESIAN_READS
	if ((init == 1) || (restart == 0)) 
	//if ((init == 1) || (sampleCount >= iteration) || (restart == 1)) 
	{
		sampleCount = 0;
		init = 0;
		for (int i = 0; i < BAYESIAN_STATE_SIZE; i++) 
		{
			prob_array[i] = en;
		}
	}

	/* ===== Start Predicting ======== */
	double mean_a = 0.9581686069037303;
	double mean_b = 0.7625882833237847;
	double std_a = 0.009280269184555247;
	double std_b = 0.48783070995817385;

	for (int i = 0; i < BAYESIAN_STATE_SIZE; i++) 
	{
		double mean = mean_a * (double)state[i] + mean_b;
		double std = std_a * (double)state[i] + std_b;
		double variance = pow((std), 2.0);
		double prob = ((1.0 / (std * sqrt(2.0 * M_PI))) * (pow(M_E, -1.0 * ((pow((actual_distance - mean), 2.0)) / (variance * 2.0)))));
		prob_array[i] = prob * prob_array[i];
	}
	normalize_array(prob_array, BAYESIAN_STATE_SIZE);
	sampleCount++;

	/* find most confident and return value */
	for (i = 0; i < BAYESIAN_STATE_SIZE; i++)
	{
		if (prob_array[i] > find_max)
		{
			find_max = prob_array[i];
			at_idx = i;
		}
	}

	// Generate the random numbers that follows the Gausian Distribution
	return mean_a * (double)state[at_idx] + mean_b;;
}

double randnorm(double mu, double sigma)
{
	double U1, U2, W, mult;
	static double X1, X2;
	static int call = 0;

	if (call == 1)
	{
		call = !call;
		return (mu + sigma * (double)X2);
	}

	do
	{
		U1 = -1 + ((double)rand() / RAND_MAX) * 2;
		U2 = -1 + ((double)rand() / RAND_MAX) * 2;
		W = pow(U1, 2) + pow(U2, 2);
	} while (W >= 1 || W == 0);

	mult = sqrt((-2 * log(W)) / W);
	X1 = U1 * mult;
	X2 = U2 * mult;

	call = !call;

	return (mu + sigma * (double)X1);
}

double generate_characterized_sensor_read(double distance) 
{
	// Constant to calculate to mean and standard deviation from the given distance
	static double mean_a = 0.9581686069037303;
	static double mean_b = 0.7625882833237847;
	static double std_a = 0.009280269184555247;
	static double std_b = 0.48783070995817385;

	// Calculate the mean and std for the Gausian Distribution
	double mean = mean_a * distance + mean_b;
	double std = std_a * distance + std_b;

	// Generate the random numbers that follows the Gausian Distribution
	return randnorm(mean, std);
}

double generate_characterized_sensor_read_with_bayesian(double distance, double *probability_array, int num_reads) 
{
	return make_bayesian_prediction(probability_array, generate_characterized_sensor_read(distance), num_reads); 
}
