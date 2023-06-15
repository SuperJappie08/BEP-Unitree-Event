#!/usr/bin/python3
import csv
import matplotlib.pyplot as plt
from operator import itemgetter


def plot_timeline(dataset, **kwargs):
    outpath = kwargs.pop('savefig', None)  # Save the figure as an SVG
    colors  = kwargs.pop('colors', {})     # Plot the colors for the series.
    series  = set([])                      # Figure out the unique series

    # Bring the data into memory and sort
    dataset = sorted(list(dataset), key=itemgetter(0))

    # Make a first pass over the data to determine number of series, etc.
    for _, source, category in dataset:
        series.add(source)
        if category not in colors:
            colors[category] = 'k'

    # Sort and index the series
    series  = sorted(list(series))

    # Create the visualization
    x = []  # Scatterplot X values
    y = []  # Scatterplot Y Values
    c = []  # Scatterplot color values

    # Loop over the data a second time
    for timestamp, source, category in dataset:
        x.append(timestamp)
        y.append(series.index(source))
        c.append(colors[category])

    plt.figure(figsize=(20,4))
    plt.title(kwargs.get('title', "Timeline Plot"))
    plt.ylim((-1,len(series)))
    plt.xlim((dataset[0][0], dataset[-1][0]+1000))
    plt.yticks(range(len(series)), series)
    plt.scatter(x, y, color=c, alpha=0.1, s=10)

    if outpath:
        return plt.savefig(outpath, format='svg', dpi=1200)

    return plt
