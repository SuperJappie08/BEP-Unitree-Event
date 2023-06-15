import rosbag
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

# Parameters to be adjusted
bag_file = '/media/remi/GamerPort/BEP/Datasets/Original/Main_Entrance_3mE.bag'
end_time_in_seconds = 0  # relevant for removing noise, how many seconds of the recording are to be processed and put
# in a new bag. If it's 0, it does the whole recording

# Don't adjust these parameters
end_time = end_time_in_seconds * 10 ** 9
image_size = (260, 346)
number_of_pixels = 260 * 346
noise_percent = 50


# Returns the filename to be used for the processed bags
def return_new_bag_name(bagfile):
    part_to_replace = ".bag"
    replacement = "_Processed.bag"
    new_string = bagfile.replace(part_to_replace, replacement)
    return new_string


# Checks whether the number of events is too high
def check_if_noisy(num_events):
    return (num_events / number_of_pixels) * 100 > noise_percent


def return_percentage_of_noisy_frames(bag_file):
    bag = rosbag.Bag(bag_file)
    num_entries = bag.get_message_count()
    noisy_frames_count = 0
    frames_count = 0
    bag_reader = bag.read_messages()
    for topic, msg, timestamp in tqdm(bag_reader, total=num_entries):
        if topic != '/dvs/events':
            continue
        frames_count += 1
        is_noisy = check_if_noisy(sum(1 for _ in msg.events))
        if is_noisy:
            noisy_frames_count += 1

    return (noisy_frames_count / frames_count) * 100


# Returns pixel variation ratio per frame as a list
def return_percentage_per_frame(bag_file):
    bag = rosbag.Bag(bag_file)
    num_entries = bag.get_message_count()
    percentages = [(sum(1 for _ in msg.events) / number_of_pixels) * 100
                   for topic, msg, timestamp in tqdm(bag.read_messages(), total=num_entries, desc='Processing')
                   if topic == '/dvs/events']

    bag.close()
    return percentages


# Removes noisy frames and uses the most recent frame instead, outputs the result as a new bag with just the event topic
def remove_noise_previous(bag_file):
    previous_message = 0
    bag = rosbag.Bag(bag_file)
    num_entries = bag.get_message_count()

    new_bag_file = return_new_bag_name(bag_file)
    new_bag = rosbag.Bag(new_bag_file, 'w')  # Open the new bag file for writing
    bag_reader = bag.read_messages()
    t0 = next(bag_reader)[2]
    for topic, msg, timestamp in tqdm(bag_reader, total=num_entries):
        if topic != '/dvs/events':
            continue
        timestamp -= t0
        if timestamp.to_nsec() > end_time != 0:
            break
        current_message = msg
        events_batch = current_message.events
        num_events = len(events_batch)
        is_noisy = check_if_noisy(num_events)
        if previous_message != 0 and is_noisy:
            new_bag.write(topic, previous_message, timestamp)  # Write previous message to new bag
        else:
            new_bag.write(topic, current_message, timestamp)  # Write current message to new bag
            previous_message = current_message
    new_bag.close()


# Generates a boxplot and a histogram for the given pixel variation ratio's of every frame
def create_plots(bag_file, data):
    # Create a figure with two subplots
    fig, (ax_boxplot, ax_histogram) = plt.subplots(1, 2, figsize=(10, 4))

    # Plot the boxplot on the left subplot
    ax_boxplot.boxplot(data)
    ax_boxplot.set_title('Boxplot')

    # Plot the histogram on the right subplot
    ax_histogram.hist(data, bins=20, edgecolor="black")
    ax_histogram.set_title('Histogram')

    # Adjust spacing between subplots
    fig.tight_layout()

    fig.suptitle(bag_file)
    fig.subplots_adjust(top=0.85)

    # Display the figure
    plt.show()


# Generates 5 number summary for the pixel variation ratio's of every frame
def generate_five_number_summary(data):
    minimum = np.min(data)
    q1 = np.percentile(data, 25)
    median = np.median(data)
    q3 = np.percentile(data, 75)
    maximum = np.max(data)
    return minimum, q1, median, q3, maximum


