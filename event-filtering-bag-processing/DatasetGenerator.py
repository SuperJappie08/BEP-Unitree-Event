import rosbag


# This method reads a bag file and outputs its entries into a list with format (timestamp, topic, colour)
def read_bag(bag_file, start_time, end_time):
    entries = []
    first = True
    shift = False
    t0 = 0
    bag = rosbag.Bag(bag_file)
    for topic, msg, timestamp in bag.read_messages():
        # if topic in topics.keys():
        if first: # just don't think about the unga-bunga :)
            t0 = timestamp
            first = False

        timestamp = timestamp - t0

        if start_time * 10 ** 9 <= timestamp.to_nsec():
            entries.append((timestamp, topic, 'red'))
            if topic == '/dvs/events':
                # unpack the arrays, save their timestamps and subtract t0 from them
                events = msg.events
                for event in events:
                    ts = event.ts
                    entries.append((ts - t0, topic, 'red'))
            else:
                entries.append((timestamp, topic, 'red'))

        if timestamp.to_nsec() > end_time * 10 ** 9:
            return entries

    return sorted(entries, key=lambda tup: tup[0])




