import rosbag

filename = "/home/shunki/ros_ws/ros1/ohrc_ws/src/OpenHRC/ohrc_imitation_learning/data/data_2023-01-17-18-04-44.bag"
with rosbag.Bag(filename) as bag:
    topics = bag.get_type_and_topic_info()[1].keys()
    print(topics)
