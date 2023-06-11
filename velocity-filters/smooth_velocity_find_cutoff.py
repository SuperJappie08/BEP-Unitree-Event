# -*- coding: utf-8 -*-
"""
Created on Thu Jun  8 11:20:49 2023

@author: Erik van Huffelen
"""
from __future__ import annotations
import scipy.signal
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.highlevel import AnyReader
from typing import TYPE_CHECKING
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag1 import Writer
from rosbags.serde import cdr_to_ros1, serialize_cdr
from rosbags.typesys.types import geometry_msgs__msg__Vector3 as Vector3
from rosbags.typesys.types import std_msgs__msg__Float32 as Float32

if TYPE_CHECKING:
    from pathlib import Path

path = 'C:/Users/Erik van Huffelen/Documents/Uni/Doggo/Filtered raw data/IO gang boven/BEP-TEST_2023-06-08-11-56-37.bag'
def process_bag(src: Path) -> None:
    """Register contained messages types before processing bag.

    Args:
        src: Bag to process.

    """
    with Reader(src) as reader:
        typs = {}
        for conn in reader.connections:
            typs.update(get_types_from_msg(conn.msgdef, conn.msgtype))
        register_types(typs)

        # Now all message types used in the bag are registered
        # for conn, timestamp, data in reader.messages():
        #     ...
process_bag(path)

empty_list = list()# np.empty((1,2))
with Reader(path) as reader:
    # topic and msgtype information is available on .connections list
    #for connection in reader.connections:
       #print(connection.topic, connection.msgtype)

    # messages() accepts connection filters
    connections = [x for x in reader.connections if x.topic == '/bep_controller/robot_state']
    for connection, timestamp, rawdata in list(reader.messages(connections=connections)):
        msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
        empty_list.append([timestamp, *msg.velocity, msg.yawSpeed])
        

elist = np.array(empty_list)
starttime = elist[0, 0]   
elist[:,0] -= starttime 
elist[:,0] /= 1e9                   #van nanoseconde naar seconde
#plt.plot(elist[:,0], elist[:,1:3], label=['x', 'y'])  #1 = x-richting, 2 = y-richting (negatief is rechts), 3 = z-richting
#plt.legend()
#plt.show()
         

# read ECG data from the WAV file
sampleRate = 500
times = np.arange(len(elist[:,1]))/sampleRate

# apply a 3-pole lowpass filter
b, a = scipy.signal.butter(3, 0.0025)
c, d = scipy.signal.butter(3, 0.005)            #butter filters with a 3-pole lowpass structure
filteredx = scipy.signal.filtfilt(b, a, elist[:,1])     #apply filters to the x,y,z,yaw data
filteredy = scipy.signal.filtfilt(b, a, elist[:,2])
filteredz = scipy.signal.filtfilt(b, a, elist[:,3])
filteredyawSpeed = scipy.signal.filtfilt(c, d, elist[:,4])
filtered = [filteredx,filteredy,filteredz, filteredyawSpeed]        #put data back into one array
#b, a = scipy.signal.butter(3, 0.005)               prima setting
#filtered = scipy.signal.filtfilt(b, a, data)

# plot the original data next to the filtered data

# plt.figure(figsize=(10, 4))
cutoff_index = np.where((times>34.999)*(times<35.001))          #look at plot and find time, put the small margin into this
cutoff_timestamp = empty_list[cutoff_index[0][0]][0]
print(cutoff_timestamp)
plt.figure(figsize=(25, 6), dpi=80)


plt.plot(times, elist[:,1], label="x Signal with Noise")        #plot the signals and the filtered signals
plt.plot(times, filtered[0], label="Filtered x Signal")
plt.xticks(np.arange(min(times), max(times)+1, 0.5))
plt.title('x')
plt.legend()
plt.grid()
#plt.margins(0, .5)
# plt.tight_layout()
#plt.show()
#fig.savefig('C:/Users/Erik van Huffelen/Documents/Uni/Doggo/Filtered data/Saved plots/Spiegel lift.png')

# raise TypeError()
#create writer instance and open for writing

#below is the code for when writing a bag file from the filtered code, use other file instead 
"""
with Writer(path.replace("TEST", "FILTER")) as writer:
    # add new connection
    topic = '/velocity_filtered'
    msgtype = Vector3.__msgtype__
    
    topic_yaw = '/yaw_speed_filtered'
    msgtype_yaw = Float32.__msgtype__
    
    connection = writer.add_connection(topic, msgtype, latching=True)
    connection_yaw = writer.add_connection(topic_yaw, msgtype_yaw, latching=True)
    
    for index,timestamp in enumerate(l[0] for l in empty_list):
        # serialize and write message
        message = Vector3(filteredx[index], filteredy[index], filteredz[index])
        writer.write(connection, timestamp, cdr_to_ros1(serialize_cdr(message, msgtype), msgtype))
        message_yaw = Float32(filteredyawSpeed[index])
        writer.write(connection_yaw, timestamp, cdr_to_ros1(serialize_cdr(message_yaw, msgtype_yaw), msgtype_yaw))
"""
    
print("Done")