The scripts provided in this folder enable you to process the dataset, removing noise, and generate vizualizations of the data in the dataset.

The only "rule" for these scripts is to only adjust the parameters that are marked to be adjusted, as altering some of the "internal" parameters without knowing what they do in the code could break it.

1. DatasetCleanup.py contains the methods required to remove noise from the bag and to generate the pixel variation ratio's (see the method section of the report) for each frame of the provided dataset
2. ReadAndUnpackbBag.py allows you to create gifs of the events as they occur, gaining a video feed of what the event based camera is seeing
3. DatasetGenerator.py and TimelineGenerator.py can be used together to create a timeline like you can see in the results section of the report. An example of how to do this can be found in "MakingATimeline.py"

In order to use these scripts, you need the rosbag ros package. This requires you to install this package in ros, and use ros' python interpreter.
Alternatively, you can use your own python interpreter, provided you correctly source the rosbag package.
