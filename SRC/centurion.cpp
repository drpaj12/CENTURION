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
#include <sstream>

#include "types.h"
#include "globals.h"
#include "utils.h"

#include "argparse.hpp"
#include "read_xml_config_file.h"

/* globals */
global_args_t global_args;

sim_system_t sim_system;
environment_t environment;
agent_groups_t agent_groups;
atons_t atons;
communication_stack_t comm_stack;

FILE *f_log_out;
FILE *f_debug_out;

/* prototypes */
void get_options(int argc, char** argv) ;

int main(int argc, char **argv)
{
	printf("--------------------------------------------------------------------\n");
	printf("Welcome the centurion simulator\n");
	printf("Email: jamieson.peter@gmail.com\n\n");

	/* get the command line options */
	get_options(argc, argv);

	/* read config file */
	read_config_file(global_args.config_file);

	/* check parameters */

	/* open final ouput files */
	f_debug_out = fopen(sim_system.debug_file_out, "w");
	oassert(f_debug_out != NULL);
	f_log_out = fopen(sim_system.sim_log_file_out, "w");
	oassert(f_log_out != NULL);

	/* set randomization */
	srand(sim_system.rand_seed);
	rand_float_seed(sim_system.rand_seed);
	printf("rand seed: %d\n", sim_system.rand_seed);

	/* ---- BASIC Sequential GA Executions ---- */
	if (strcmp(sim_system.simulation_type, "discrete") == 0)
	{
	}
	else
	{
		printf("Unsupported simulation type\n");
	}

	/*-------------------FREE_PROBLEM------------------*/
	/* free the problem */
	fclose(f_debug_out);
	fclose(f_log_out);

	return 1;
}

/*---------------------------------------------------------------------------------------------
 * (function: get_options)
 *-------------------------------------------------------------------------------------------*/
void get_options(int argc, char** argv) 
{
	auto parser = argparse::ArgumentParser(argv[0]);

	global_args.program_name = parser.prog();

	parser.add_argument(global_args.config_file, "-c")
		.help("Configuration file")
		.metavar("XML_CONFIGURATION_FILE")
		.default_value("no_config_file.xml")
		;

	parser.add_argument(global_args.output_file, "-o")
		.help("Output file path")
		.default_value("default_out.txt")
		.metavar("OUTPUT_FILE_PATH")
		;

	parser.add_argument(global_args.test_file, "-t")
		.help("Test Output file path")
		.default_value("default_test_out.txt")
		.metavar("TEST_FILE_PATH")
		;

	parser.add_argument(global_args.show_help, "-h")
		.help("Display this help message")
		.action(argparse::Action::HELP)
		;

	parser.parse_args(argc, argv);
}
