#!/usr/bin/python3
import rosbag
import imageio
import numpy as np
from tqdm import tqdm
from PIL import Image
bag_file = '/media/remi/GamerPort/BEP/Datasets/Processed/Main_Entrance_3mE_Processed.bag'

noise_percent = 60
image_size = (260, 346)


# Reads the bag and returns a gif containing the events per frame
def read_bag_events_only(start_time, end_time):
    bag = rosbag.Bag(bag_file, 'r')
    num_entries = bag.get_message_count()
    bag_reader = bag.read_messages()  # generator
    t0 = next(bag_reader)[2]
    start_time_nsc = start_time * 10 ** 9
    end_time_nsc = end_time * 10 ** 9

    # Camera dimensions are 346 x 260 (X times Y)
    # event.x goes from 0 to 345 and event.y goes from 0 to 259
    default_value = 130
    default_array = np.full(image_size, default_value).astype('uint8')
    image_arrays = []

    # update the current image based on the event_batches that occur
    for topic, msg, timestamp in tqdm(bag_reader, total=num_entries):
        # only check the davis topics
        if topic != '/dvs/events':
            continue

        # guard
        timestamp -= t0
        if timestamp.to_nsec() > end_time_nsc != 0:
            break
        if timestamp.to_nsec() >= start_time_nsc:
            # For each batch of events, make an empty greyscale image
            current_image_array = default_array.copy()
            events_batch = msg.events
            x_positions = np.array([event.x for event in events_batch])
            y_positions = np.array([event.y for event in events_batch])
            polarity = np.array([event.polarity for event in events_batch])
            new_intensity_values = np.where(polarity, 255, 0)

            current_image_array[y_positions, x_positions] = new_intensity_values  # vectorization
            image_arrays.append(np.array(current_image_array))  # make a snapshot of the current image

    image_list = [Image.fromarray(image_array, mode='L') for image_array in image_arrays]
    image_list[0].save('events only.gif', save_all=True, append_images=image_list[1:], duration=40, loop=0)


# Checks whether a frame was noisy (as defined by the noise percentage)
def check_if_noisy(num_events):
    number_of_pixels = 260 * 346
    return (num_events / number_of_pixels) * 100 > noise_percent


# Make a mp4 containing only noisy frames
def retrieve_noisy_frames(start_time, end_time):
    bag = rosbag.Bag(bag_file, 'r')
    num_entries = bag.get_message_count()
    bag_reader = bag.read_messages()  # generator
    t0 = next(bag_reader)[2]
    start_time_nsc = start_time * 10 ** 9
    end_time_nsc = end_time * 10 ** 9

    # Camera dimensions are 346 x 260 (X times Y)
    # event.x goes from 0 to 345 and event.y goes from 0 to 259

    default_value = 130
    default_array = np.full(image_size, default_value).astype('uint8')
    image_arrays = []

    # update the current image based on the event_batches that occur
    for topic, msg, timestamp in tqdm(bag_reader, total=num_entries):
        # only check the davis topics
        if topic != '/dvs/events':
            continue

        # guard
        timestamp -= t0
        if timestamp.to_nsec() > end_time_nsc != 0:
            break
        if timestamp.to_nsec() >= start_time_nsc:
            # For each batch of events, check if its noisy
            events_batch = msg.events
            if not check_if_noisy(len(events_batch)):
                continue

            # Paint the grey canvas and take a snapshot
            current_image_array = default_array.copy()
            events_batch = msg.events
            x_positions = np.array([event.x for event in events_batch])
            y_positions = np.array([event.y for event in events_batch])
            polarity = np.array([event.polarity for event in events_batch])
            new_intensity_values = np.where(polarity, 255, 0)
            current_image_array[y_positions, x_positions] = new_intensity_values  # vectorization
            image_arrays.append(np.array(current_image_array))  # make a snapshot of the current image

    image_list = [Image.fromarray(image_array, mode='L') for image_array in image_arrays]
    imageio.mimsave('video.mp4', image_list, fps=25)


# Reconstructs the greyscale image by accumulating the observed events on the first greyscale image
def read_bag_greyscale_reconstruction(start_time, end_time):
    contrast = 75
    bag = rosbag.Bag(bag_file)
    num_entries = bag.get_message_count()
    bag_reader = bag.read_messages()  # generator
    start_time_nsc = start_time * 10 ** 9
    end_time_nsc = end_time * 10 ** 9

    # Camera dimensions are 346 x 260 (X times Y)
    # event.x goes from 0 to 345 and event.y goes from 0 to 259
    image_arrays = []

    # find the first greyscale image
    found = False
    current_image_array = []
    t0 = next(bag_reader)[2]

    while not found:
        topic, msg, timestamp = next(bag_reader)
        timestamp -= t0
        if topic == '/dvs/image_raw' and timestamp.to_nsec() > start_time_nsc:
            # convert binary image data into 1D image array
            one_dimensional_image = np.frombuffer(msg.data, dtype=np.uint8).copy()
            current_image_array = np.reshape(one_dimensional_image, image_size)
            found = True

    image_arrays.append(current_image_array)

    # update the current image based on the event_batches that occur
    for topic, msg, timestamp in tqdm(bag_reader, total=num_entries):
        timestamp = timestamp - t0
        if timestamp.to_nsec() > end_time_nsc != 0:
            break
        # only check the davis topics
        if topic != '/dvs/events':
            continue
        # For each batch of events, make an empty greyscale image
        events_batch = msg.events
        x_positions = np.array([event.x for event in events_batch])
        y_positions = np.array([event.y for event in events_batch])
        polarity = np.array([event.polarity for event in events_batch])
        intensity_changes = np.where(polarity, contrast, -contrast)
        new_values = current_image_array[y_positions, x_positions] + intensity_changes

        current_image_array[y_positions, x_positions] = np.clip(new_values, 0, 255)
        image_arrays.append(np.array(current_image_array))  # make a snapshot of the current image

    image_list = [Image.fromarray(image_array, mode='L') for image_array in image_arrays]
    image_list[0].save('greyscale reconstruction.gif', save_all=True, append_images=image_list[1:], duration=40, loop=0)


def read_bag_image_raw(start_time, end_time):
    bag = rosbag.Bag(bag_file)
    num_entries = bag.get_message_count()
    bag_reader = bag.read_messages()  # generator
    start_time_nsc = start_time * 10 ** 9
    end_time_nsc = end_time * 10 ** 9

    # Camera dimensions are 346 x 260 (X times Y)
    # event.x goes from 0 to 345 and event.y goes from 0 to 259
    image_arrays = []
    current_image_array = np.zeros(image_size)
    t0 = next(bag_reader)[2]

    for topic, msg, timestamp in tqdm(bag_reader, total=num_entries):
        timestamp = timestamp - t0
        if topic != '/dvs/image_raw':
            continue

        if timestamp.to_nsec() > end_time_nsc != 0:
            break

        if timestamp.to_nsec() >= start_time_nsc:
            # convert binary image data into 1D image array
            one_dimensional_image = np.frombuffer(msg.data, dtype=np.uint8)
            if len(one_dimensional_image) == 89960:
                current_image_array = np.reshape(one_dimensional_image, image_size)
            image_arrays.append(current_image_array)

    image_list = [Image.fromarray(image_array, mode='L') for image_array in image_arrays]
    image_list[0].save('raw image.gif', save_all=True, append_images=image_list[1:], duration=40, loop=0)


# Convenience function for applying all methods at the same time
def read_all(start_time, end_time):
    print('Events only!')
    read_bag_events_only(start_time, end_time)
    print('Noisy frames only!')
    retrieve_noisy_frames(start_time, end_time)
    print('Greyscale reconstruction!')
    read_bag_greyscale_reconstruction(start_time, end_time)
    print('Image raw!')
    read_bag_image_raw(start_time, end_time)


read_all(0, 30)
