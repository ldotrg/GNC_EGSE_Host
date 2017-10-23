#!/usr/bin/env python
import re
import numpy
def counting_actual_size(file, output):
	body_size = 0
	not_counting_size = 0
	# Read the first line
	line = file.readline()
	while line:
		if line[:1] == "#" or line[:] == "\0" or line[:] == "\n":
			not_counting_size +=1
		else:
			output.write(line)
		line = file.readline()
		body_size += 1
	body_size -= not_counting_size
	print("%s lines = %d" % (file.name, body_size))
	return body_size

file_input = open("logg.txt","r")
file_out = open("output.txt","w+")
#plot_file = open("plot_data_input.txt", "w+")
logger_size = counting_actual_size(file_input, file_out)
file_input.close()
file_out.close()


with open("output.txt", "r") as file_out:
	data = file_out.readlines()

trip_time_ms_list = []
for idx, element in enumerate(data):
	if element.find("sys_enter: NR 511") > 0:
		temp_list = re.findall("\d+\.\d+", data[idx - 2])
		start_time = float(temp_list[0])
		temp_list = re.findall("\d+\.\d+", data[idx + 19])
		end_time = float(temp_list[0])
		trip_time_ms_list.append((end_time - start_time) * 1000)
print("22 lines per cycle count\n")

print("|Round trip count | %d |" % (len(trip_time_ms_list)))
print("|Avg| %f ms |" % (sum(trip_time_ms_list)/len(trip_time_ms_list)))
print("|Max| %f ms |" % (max(trip_time_ms_list)))
print("|Min| %f ms |" % (min(trip_time_ms_list)))
print("|sigma| %f  |" % (numpy.std(trip_time_ms_list)))

with open("plot_data_input.txt", "w") as file_out:
    for idx in range(len(trip_time_ms_list)):
        file_out.write(str(idx+1) + " " + str(trip_time_ms_list[idx]) +"\n")

print "Please use the following command to plot:"
print "gnuplot -e 'in=\"plot_data_input.txt\";out=\"output1.png\";gtitle=\"EGSE RX CAN and TX RS422 driver IO time\"; \
       count=%d;max=%f;avg=%f;sigma=%f' plot.gp" \
       % (len(trip_time_ms_list),max(trip_time_ms_list),sum(trip_time_ms_list)/len(trip_time_ms_list),numpy.std(trip_time_ms_list))