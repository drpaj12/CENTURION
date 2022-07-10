#!/usr/bin/python

import subprocess

def run_program(path):
    p = subprocess.Popen(path, shell=True, stdout=subprocess.PIPE)
    p.communicate()[0]

    if (p.returncode > 1):
        return False
    return True

# Program runs all the programs in the directory
delete_src_first = 'rm -rf /home/robot/DEVELOPMENT/RESEARCH/SUPERVISION_AND_PROJECTS/PROGRAMS_PROJECTS_DEVELOPMENT/ROBOT_AGENT_SYSTEM/CENTURION_/CENTURION/SRC/*'
run_program(delete_src_first)
copy_to = 'cp -r /home/robot/SIMULATION_WORK/CENTURION/SRC/* /home/robot/DEVELOPMENT/RESEARCH/SUPERVISION_AND_PROJECTS/PROGRAMS_PROJECTS_DEVELOPMENT/ROBOT_AGENT_SYSTEM/CENTURION_/CENTURION/SRC'
run_program(copy_to)
copy_to = 'cp -r /home/robot/SIMULATION_WORK/CENTURION/SCRIPTS_UTILS/* /home/robot/DEVELOPMENT/RESEARCH/SUPERVISION_AND_PROJECTS/PROGRAMS_PROJECTS_DEVELOPMENT/ROBOT_AGENT_SYSTEM/CENTURION_/CENTURION/SCRIPTS_UTILS'
run_program(copy_to)
copy_to = 'cp -r /home/robot/SIMULATION_WORK/CENTURION/SANDBOX/* /home/robot/DEVELOPMENT/RESEARCH/SUPERVISION_AND_PROJECTS/PROGRAMS_PROJECTS_DEVELOPMENT/ROBOT_AGENT_SYSTEM/CENTURION_/CENTURION/SANDBOX'
run_program(copy_to)


