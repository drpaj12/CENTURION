#!/usr/bin/python

import subprocess

def run_program(path):
    p = subprocess.Popen(path, shell=True, stdout=subprocess.PIPE)
    p.communicate()[0]

    if (p.returncode > 1):
        return False
    return True

# Program runs all the programs in the directory
copy_to = 'cp -r /home/robot/SIMULATION_WORK/CENTURION/* /home/robot/DEVELOPMENT/RESEARCH/SUPERVISION_AND_PROJECTS/PROGRAMS_PROJECTS_DEVELOPMENT/ROBOT_AGENT_SYSTEM/CENTURION_/CENTURION/'

run_program(copy_to)

