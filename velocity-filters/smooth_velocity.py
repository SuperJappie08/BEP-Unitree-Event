# -*- coding: utf-8 -*-
"""
Created on Thu Jun  8 11:20:49 2023

@author: Erik van Huffelen
"""
#all imports below
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
    
#input the path to the file below
path = 'C:/Users/Erik van Huffelen/Documents/Uni/Doggo/Filtered raw data/Spiegel lift/BEP-TEST_2023-06-07-12-18-11.bag'

#define the function to process the bag, this ensures all the message types are 'known'
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
process_bag(path)           #use the function to actually process the desired bag

#reads the information from the bag and puts it into the empty_list
empty_list = list()# np.empty((1,2))
with Reader(path) as reader:
    # topic and msgtype information is available on .connections list
    #for connection in reader.connections:
       #print(connection.topic, connection.msgtype)

    # messages() accepts connection filters
    connections = [x for x in reader.connections if x.topic == '/bep_controller/robot_state']
    for connection, timestamp, rawdata in list(reader.messages(connections=connections)):
        msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
        empty_list.append([timestamp, *msg.velocity, msg.yawSpeed])     #selects the velocity and yawspeed data 
        
#transforms the data from the empty_list (in timestamp instead of seconds) to usuable data
elist = np.array(empty_list)
starttime = elist[0, 0]   
elist[:,0] -= starttime 
elist[:,0] /= 1e9                   #from nano seconds (timestamp) to seconds
#plt.plot(elist[:,0], elist[:,1:3], label=['x', 'y'])  #1 = x-richting, 2 = y-richting (negatief is rechts), 3 = z-richting
#plt.legend()
#plt.show()
         
#makes times array 
sampleRate = 500
times = np.arange(len(elist[:,1]))/sampleRate

# apply a 3-pole lowpass filter
b, a = scipy.signal.butter(3, 0.0025)       #defines the filters to be applied to the signals
c, d = scipy.signal.butter(3, 0.005)
filteredx = scipy.signal.filtfilt(b, a, elist[:,1])     #apply filters to signals of speed in x y and z and yaw
filteredy = scipy.signal.filtfilt(b, a, elist[:,2])
filteredz = scipy.signal.filtfilt(b, a, elist[:,3])
filteredyawSpeed = scipy.signal.filtfilt(c, d, elist[:,4])
filtered = [filteredx,filteredy,filteredz, filteredyawSpeed]
#b, a = scipy.signal.butter(3, 0.005)               prima setting
#filtered = scipy.signal.filtfilt(b, a, data)

# plot the original data next to the filtered data

# plt.figure(figsize=(10, 4))
fig, ax = plt.subplots(2,2, figsize=(10,8))

labels = ['x', 'y', 'z', 'yaw speed']
for i in range(4):
    ax[i%2, i//2].plot(times, elist[:,i+1], label=f"{labels[i]} Signal with Noise")
    ax[i%2, i//2].plot(times, filtered[i], label=f"Filtered {labels[i]} Signal")
    ax[i%2, i//2].set_title(labels[i])
    ax[i%2, i//2].legend()
plt.margins(0, .5)
# plt.tight_layout()
#plt.show()
fig.savefig('C:/Users/Erik van Huffelen/Documents/Uni/Doggo/Filtered data/Saved plots/Spiegel lift.png')


#creates writer for the new bag with the filtered signals 
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