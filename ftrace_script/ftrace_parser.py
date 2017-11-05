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
### CONSTANT ###
IRQ_NUM_PER_RX_CAN_PACKET = 7
SYSCALL_NUM_PER_ROUND_TRIP = 7
SERIAL_PORT_START_TX_CNT = 172
file_input = open("logg.txt","r")
file_out = open("output.txt","w+")
#plot_file = open("plot_data_input.txt", "w+")
logger_size = counting_actual_size(file_input, file_out)
file_input.close()
file_out.close()


with open("output.txt", "r") as file_out:
	data = file_out.readlines()

trip_time_ms_list = []
abnormal_cnt = 0
irq_count = 0
## 1. Normal log: 1st pcan_pci_irqhandler, 2nd NR511
## 2. Reorder log: 1st NR511 , 2nd pcan_pci_irqhandler
for idx, element in enumerate(data):
	if element.find("sys_enter: NR 511") > 0:
		temp_list = re.findall("\d+\.\d+", data[idx - 1])
		start_time = float(temp_list[0])
		temp_list = re.findall("\d+\.\d+", data[idx + SERIAL_PORT_START_TX_CNT + 6])
		end_time = float(temp_list[0])
		duration = (end_time - start_time) * 1000
		if duration > 48:
			abnormal_cnt += 1;
			temp_list = re.findall("\d+\.\d+", data[idx])
			start_time = float(temp_list[0])
			temp_list = re.findall("\d+\.\d+", data[idx + SERIAL_PORT_START_TX_CNT + 7])
			end_time = float(temp_list[0])
			duration_fixed = (end_time - start_time) * 1000
			print "[%d]Line: %d, duration %f => %f" % (abnormal_cnt, idx+1, duration, duration_fixed)
			trip_time_ms_list.append(duration_fixed)
			continue
		trip_time_ms_list.append(duration)

# for idx, element in enumerate(data):
# 	if element.find("irq_handler_entry: irq=16 name=pcan") > 0:
# 		irq_count += 1
# 		if irq_count == 1:
# 			temp_list = re.findall("\d+\.\d+", data[idx])
# 			start_time = float(temp_list[0])
# 		if irq_count == 7:
# 			temp_list = re.findall("\d+\.\d+", data[idx + 172])
# 			end_time = float(temp_list[0])
# 			duration = (end_time - start_time) * 1000
# 			if duration > 3:
# 				abnormal_cnt += 1;
# 				print "[%d]output.txt Line: %d, time duration:%f" % (abnormal_cnt, idx+1, duration)
# 			trip_time_ms_list.append(duration)
# 			irq_count = 0

# for idx, element in enumerate(data):
# 	if element.find("sys_enter: NR 501") > 0:
# 		temp_list = re.findall("\d+\.\d+", data[idx - IRQ_NUM_PER_RX_CAN_PACKET])
# 		start_time = float(temp_list[0])
# 		temp_list = re.findall("\d+\.\d+", data[idx])
# 		end_time = float(temp_list[0])
# 		duration = (end_time - start_time) * 1000
# 		if duration > 3:
# 			abnormal_cnt += 1;
# 			print "[%d]output.txt Line: %d, time duration:%f" % (abnormal_cnt, idx+1, duration)
# 		trip_time_ms_list.append(duration)

print("|Round trip count | %d |" % (len(trip_time_ms_list)))
print("|Avg| %f ms |" % (sum(trip_time_ms_list)/len(trip_time_ms_list)))
print("|Max| %f ms |" % (max(trip_time_ms_list)))
print("|Min| %f ms |" % (min(trip_time_ms_list)))
print("|sigma| %f  |" % (numpy.std(trip_time_ms_list)))

with open("plot_data_input.txt", "w") as file_out:
    for idx in range(len(trip_time_ms_list)):
        file_out.write(str(idx+1) + " " + str(trip_time_ms_list[idx]) +"\n")

print "Please use the following command to plot:"
print "gnuplot -e 'in=\"plot_data_input.txt\";out=\"output1.png\";gtitle=\"EGSE RX CAN and TX RS422 driver IO time\";\
count=%d;max=%f;avg=%f;sigma=%f' plot.gp" \
% (len(trip_time_ms_list),max(trip_time_ms_list),sum(trip_time_ms_list)/len(trip_time_ms_list),numpy.std(trip_time_ms_list))