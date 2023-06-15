#!/usr/bin/python3
import csv
import numpy
import DatasetGenerator
import TimelineGenerator
# Example of how to make a timeline

# Parameters to be adjusted
bag_file = '/media/remi/GamerPort/BEP/Datasets/Original/Bicycle_Rack.bag'
bag1_colour = 'red'
start_time = 0
end_time = 1
# -----------------------------------------

# Reading the bag files (don't adjust this)
entries = DatasetGenerator.read_bag(bag_file, start_time, end_time)

colors = {'red': 'r', 'blue': 'b', 'green': 'g'}
plt = TimelineGenerator.plot_timeline([
        (entry[0].to_nsec(), entry[1], entry[2])
        for entry in entries
    ], colors=colors)
plt.show()
