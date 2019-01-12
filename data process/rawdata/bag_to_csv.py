'''
This script saves each topic in a bagfile as a csv.

Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory
www.speal.ca

Supervised by Professor Inna Sharf, Professor Meyer Nahon 

'''

import rosbag, sys, csv
import time
import string
import os #for file management make directory
import shutil #for file management, copy file
import numpy as np
import scipy.io as sio

class FullStateDecoder:
	def __init__(self):
		# 1-D array
		self.time = []

		# 4-D array time x drone x (pos,vel,acc) x (x,y,z)
		self.state = []
		# 2-D array time x drone
		self.bfeasible = []

	def record(self, msg):
		self.time.append(msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9)
		states = []
		bfeasible = []

		for fullstate in msg.fullstate:
			states.append([list(fullstate.pos), list(fullstate.vel),list(fullstate.acc)])
			bfeasible.append(fullstate.bfeasible)
		self.bfeasible.append(bfeasible)
		self.state.append(states)

	def printAll(self):
		print(np.array(self.time).shape)
		print(np.array(self.state).shape)
		print(np.array(self.bfeasible).shape)

	def saveAll(self, dir):
		folder = dir+"/full_state"
		try:  # else already exists
			os.makedirs(folder)
		except:
			pass
		sio.savemat(folder +"/time.mat", {"time_fullstate": self.time})
		sio.savemat(folder +"/fullstate.mat", {"fullstate": self.state})
		sio.savemat(folder +"/bfeasible.mat", {"bfeasible": self.bfeasible})

class FullStateRawDecoder:
	def __init__(self):
		# 1-D array
		self.time = []

		# 4-D array time x drone x (pos,vel,acc) x (x,y,z)
		self.state_raw = []

		# 4-D array time x drone x (pos,vel,acc) x (x,y,z)
		self.state_smoothen = []

		# 3-D array time x drone x (pos,vel,acc)
		self.valid = []

	def record(self, msg):
		self.time.append(msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9)
		state_raw = []
		state_smoothen = []
		valid = []

		for fullstate in msg.fullstateraw:
			state_raw.append([list(fullstate.pos), list(fullstate.vel),list(fullstate.acc)])
			valid.append([list(fullstate.valid)])
			state_smoothen.append([list(fullstate.pos_s), list(fullstate.vel_s), list(fullstate.acc_s)])
			
		self.valid.append(valid)
		self.state_raw.append(state_raw)
		self.state_smoothen.append(state_smoothen)

	def printAll(self):
		print(np.array(self.time).shape)
		print(np.array(self.state_raw).shape)
		print(np.array(self.state_smoothen).shape)
		print(np.array(self.valid).shape)

	def saveAll(self, dir):
		folder = dir+"/full_state_meas"
		try:  # else already exists
			os.makedirs(folder)
		except:
			pass
		sio.savemat(folder +"/full_state_meas.mat", {"time_meas": self.time, "raw_meas": self.state_raw, "smth_meas": self.state_smoothen, "valid_meas":self.valid})

class SwarmCmdsDecoder:
	def __init__(self):
		# time x drone
		self.time = []
		# time x drone x 4
		self.cmds = []
		self.drone_num = 0

	def record(self, msg):
		time = []
		cmds = []
		
		for cmd in msg.cmds:
			time.append(cmd.header.stamp.secs + cmd.header.stamp.nsecs*1e-9)
			cmds.append(list(cmd.values))
		if self.drone_num == 0:
			self.drone_num = len(time)
		else:
			# to exclude empty cmds published when all drones are in sleep mode
			if len(cmds) == self.drone_num:		
				self.time.append(time)
				self.cmds.append(cmds)

	def printAll(self):
		print(np.array(self.time).shape)
		print(np.array(self.cmds).shape)

	def saveAll(self, dir):
		folder = dir+"/SwarmCmds"
		try:  # else already exists
			os.makedirs(folder)
		except:
			pass
		self.printAll()
		print self.time[0]
		sio.savemat(folder + "/time.mat", {"time_cmds": self.time})
		sio.savemat(folder +"/cmds.mat", {"cmds": self.cmds})

class TimeCkPtDecoder:
	def __init__(self):
		self.dict = {}
	def record(self, msg):
		for info in msg.controllers_info:
			time = []
			for t in info.ck_pt:
				time.append(t.secs + t.nsecs * 1e-9)
			self.dict[info.name] = time
	def saveAll(self, dir):
		folder = dir+"/controller_hub_status"
		try:  # else already exists
			os.makedirs(folder)
		except:
			pass
		sio.savemat(folder + "/ck_pt.mat", self.dict)
			

class DebugDecoder:
	def __init__(self):
		self.debug_vals = {}
		self.debug_val_time = {}

	def record(self, msg):
		for log in msg.log:
			if log.name not in self.debug_vals.keys():
				self.debug_vals[log.name] = []
				self.debug_val_time[log.name] = []

			self.debug_vals[log.name].append(list(log.values))
			self.debug_val_time[log.name].append(log.stamp.secs + log.stamp.nsecs * 1e-9)
	def saveAll(self, dir):
		for name, values in self.debug_vals.items():
			folder = dir + "/debug_array"
			try:  # else already exists
				os.makedirs(folder)
			except:
				pass
			sio.savemat(folder + "/" + name +".mat", {name: values, "time":self.debug_val_time[name]})

class LogDecoder:
	def __init__(self):
		self.values = []
		self.time = []

	def record(self, msg):
		for log in msg.logdata:
			
			self.values.append(list(log.values))
			self.time.append(log.header.stamp.secs + log.header.stamp.nsecs * 1e-9)
			
	def saveAll(self, dir, name):
		folder = dir + "/log"
		try:  # else already exists
			os.makedirs(folder)
		except:
			pass
		sio.savemat(folder + "/" + name +".mat", {name: self.values, "time":self.time})


#verify correct input arguments: 1 or 2
if (len(sys.argv) > 3):
	print "invalid number of arguments:   " + str(len(sys.argv))
	print "should be 2: 'bag2csv.py' and 'bagName'"
	print "or just 1  : 'bag2csv.py'"
	sys.exit(1)
elif (len(sys.argv) == 2):
	listOfBagFiles = [sys.argv[1]]
	numberOfFiles = "1"
	print "reading only 1 bagfile: " + str(listOfBagFiles[0])
elif (len(sys.argv) == 3):
	if(sys.argv[1] == "-d"):
		target_dir = sys.argv[2]
		listOfBagFiles = ['./'+target_dir+"/"+f for f in os.listdir('./'+target_dir) if f[-4:] == ".bag"]	#get list of only bag files in current dir.
		numberOfFiles = str(len(listOfBagFiles))
		print "reading all " + numberOfFiles + " bagfiles in directory: " + target_dir + " \n"
		for f in listOfBagFiles:
			print f
		print "\n press ctrl+c in the next 3 seconds to cancel \n"
		time.sleep(3)
	else:
		print "bad argument(s): " + str(sys.argv)	#shouldnt really come up
		sys.exit(1)

count = 0
for bagFile in listOfBagFiles:
	count += 1
	print "reading file " + str(count) + " of  " + numberOfFiles + ": " + bagFile
	#access bag
	bag = rosbag.Bag(bagFile)
	bagContents = bag.read_messages()
	bagName = bag.filename


	#create a new directory
	#folder = string.rstrip(bagName, ".bag")
	folder = target_dir + "/Offboard"
	try:	#else already exists
		os.makedirs(folder)
	except:
		pass
	shutil.copyfile(bagName, folder+"/"+ "Offboard.bag")


	#get list of topics from the bag
	listOfTopics = []
	for topic, msg, t in bagContents:
		if topic not in listOfTopics:
			listOfTopics.append(topic)
	fs_dec = FullStateDecoder()
	cmd_dec = SwarmCmdsDecoder()
	debug_dec = DebugDecoder()
	ck_dec = TimeCkPtDecoder()
	fsr_dec = FullStateRawDecoder()
	log_decs = {}

	for topicName in listOfTopics:
		#Create a new CSV file for each topic
		filename = folder + '/' + string.replace(topicName, '/', '') + '.csv'
		with open(filename, 'w+') as csvfile:
			filewriter = csv.writer(csvfile, delimiter = ',')
			firstIteration = True	#allows header row
			for subtopic, msg, t in bag.read_messages(topicName):	# for each instant in time that has data for topicName
				#parse data from this instant, which is of the form of multiple lines of "Name: value\n"
				#	- put it in the form of a list of 2-element lists

				if subtopic == "full_state" or subtopic == "/full_state":
					fs_dec.record(msg)
				if subtopic == "SwarmCmds" or subtopic == "/SwarmCmds":
					cmd_dec.record(msg)
				if subtopic == "debug_array" or subtopic == "/debug_array":
					debug_dec.record(msg)
				if subtopic == "controller_hub_status" or subtopic == "/controller_hub_status":
					ck_dec.record(msg)
				if subtopic == "full_state_mea" or subtopic == "/full_state_mea":
					fsr_dec.record(msg)
				if subtopic[:3] == "log":
					name = subtopic[4:]
					if name not in log_decs.keys():
						log_decs[name] = LogDecoder()
					log_decs[name].record(msg)

				if subtopic[:4] == "/log":
					name = subtopic[5:]
					if name not in log_decs.keys():
						log_decs[name] = LogDecoder()
					log_decs[name].record(msg)		

				msgString = str(msg)
				msgList = string.split(msgString, '\n')
				instantaneousListOfData = []

				for i in range(len(msgList)):
					nameValuePair = msgList[i]
					splitPair = string.split(nameValuePair, ':')
					for i in range(len(splitPair)):	#should be 0 to 1
						splitPair[i] = string.strip(splitPair[i])
						# print(splitPair[0])
					if subtopic == "full_state" and len(splitPair) == 1:
						# print(msgList[msgList.index(nameValuePair)-1])
						# print(instantaneousListOfData[-1])
						# print(splitPair)
						for j in range(3):
							nameValuePair = msgList[i+j+1]
							splitPair = string.split(nameValuePair, ':')
							for k in range(len(splitPair)):
								splitPair[k] = string.strip(splitPair[k])
					instantaneousListOfData.append(splitPair)
				#write the first row from the first element of each pair
				if firstIteration:	# header
					headers = ["rosbagTimestamp"]	#first column header
					for pair in instantaneousListOfData:
						headers.append(pair[0])
					filewriter.writerow(headers)
					firstIteration = False
				# write the value from each pair to the file
				values = [str(t)]	#first column will have rosbag timestamp
				for pair in instantaneousListOfData:
					if len(pair) > 1:
						values.append(pair[1])
				filewriter.writerow(values)


	bag.close()
	fs_dec.saveAll(folder)
	cmd_dec.saveAll(folder)
	debug_dec.saveAll(folder)
	ck_dec.saveAll(folder)
	fsr_dec.saveAll(folder)
	for name, decoder in log_decs.items():
		decoder.saveAll(folder,name)
print "Done reading all " + numberOfFiles + " bag files."
