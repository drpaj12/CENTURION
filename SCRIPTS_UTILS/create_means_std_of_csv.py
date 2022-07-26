#Imports
import sys, getopt, csv, re
import random, time
from bs4 import BeautifulSoup
import lxml
import math

script_name = "create_means_std_of_csv.py"
inputfile = ''
outputfile = ''
try:
    opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
except getopt.GetoptError:
    print (script_name+"-i <inputfile> -o <outputfile>")
    sys.exit(2)
for opt, arg in opts:
    if opt == '-h':
        print (script_name+"-i <inputfile> -o <outputfile>")
        sys.exit()
    elif opt in ("-i", "--ifile"):
        inputfile = arg
    elif opt in ("-o", "--ofile"):
        outputfile = arg

print ('Input file is "'+ inputfile)
print ('Output file is "', outputfile)

simulation_width = 1.22
simulation_height = 0.91

points_list_to_time = []
num_experiments_idx = 0
time_sample = 0
first_run = 0
# read xml in of log file
infile = open(inputfile ,newline='')
contents = csv.reader(infile, delimiter=' ', quotechar='|')
for row in contents:
    if row == []:
        print("List is empty. Next data set")
        num_experiments_idx = num_experiments_idx + 1
        time_sample = 0
        first_run = 1
    else:
        if first_run == 0:
            points_list_to_time.append([])
        data = re.split(r'\t+', row[0])
        time = float(data[0])
        # convert to meters (/100) flip y to 0,0 being bottom left
        x = float(data[1])/100
        y = simulation_height - float(data[2])/100
        print("at time:"+str(time)+' x:'+str(x)+' y:'+str(y))
        points_list_to_time[time_sample].append((x, y))
        time_sample = time_sample + 1
    input("Press Enter to continue...")

for i in range(0, len(points_list_to_time)):
    print (points_list_to_time[i])
    # now need to compare to the ideal...
    

