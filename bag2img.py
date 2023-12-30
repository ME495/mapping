import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import matplotlib.pyplot as plt
import argparse
import numpy as np
from tqdm import tqdm


def read_file_list(filename):
    """
    Reads a trajectory from a text file.
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.
    Input:
    filename -- File name
    Output:
    dict -- dictionary of (stamp,data) tuples
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    list = [[v.strip() for v in line.split(" ") if v.strip() != ""] for line in lines if
            len(line) > 0 and line[0] != "#"]
    list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
    return dict(list)
 
 
def associate(first_list, second_list, offset, max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation
    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    """
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
 
    matches.sort()
    return matches


if __name__ == '__main__':
    # parse command line
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag_path', help='work ', default='/home/lg/rosbags/map-1230-room_converted.bag')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',
                        default=0.0)
    parser.add_argument('--max_difference',
                        help='maximally allowed time difference for matching entries (default: 0.02)', default=0.02)
    args = parser.parse_args()
    
    save_dir = args.bag_path.split('.bag')[0]
    os.makedirs(save_dir, exist_ok=True)

    rgb = os.path.join(save_dir, 'rgb')
    depth = os.path.join(save_dir, 'depth')

    if not os.path.exists(rgb):
        os.makedirs(rgb)
    if not os.path.exists(depth):
        os.makedirs(depth)

    bridge = CvBridge()

    file_handle1 = open(os.path.join(save_dir, 'depth-stamp.txt'), 'w')
    file_handle2 = open(os.path.join(save_dir, 'rgb-stamp.txt'), 'w')
        
    
    with rosbag.Bag(args.bag_path, 'r') as bag:
        for topic,msg,t in tqdm(bag.read_messages()):
            if topic == "/camera/aligned_depth_to_color/image_raw":  #depth topic
                cv_image = bridge.imgmsg_to_cv2(msg)
                timestr = "%.6f" %  msg.header.stamp.to_sec()   #depth time stamp
                image_name = timestr+ ".png"
                path = "depth/" + image_name
                file_handle1.write(timestr + " " + path + '\n')
                # print(cv_image.shape)
                # cv_image = cv2.resize(cv_image, dsize=(800,600), interpolation=cv2.INTER_NEAREST)
                cv2.imwrite(os.path.join(depth, image_name), cv_image)
            if topic == "/camera/color/image_raw":   #rgb topic
                cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
                timestr = "%.6f" %  msg.header.stamp.to_sec()   #rgb time stamp
                image_name = timestr+ ".png"
                path = "rgb/" + image_name
                file_handle2.write(timestr + " " + path + '\n')
                cv2.imwrite(os.path.join(rgb, image_name), cv_image)   
                
    file_handle1.close()
    file_handle2.close()

    # 对齐rgb和depth的时间戳
    first_file = os.path.join(save_dir, 'rgb-stamp.txt')
    second_file = os.path.join(save_dir, 'depth-stamp.txt')
    first_list = read_file_list(first_file)
    second_list = read_file_list(second_file)

    matches = associate(first_list, second_list, args.offset, args.max_difference)

    lines = []
    for a, b in matches:
        line = "%f %s %f %s\n" % (a, " ".join(first_list[a]), b - args.offset, " ".join(second_list[b]))
        lines.append(line)
        associations_path = os.path.join(save_dir, 'associations.txt')
        with open(associations_path, 'w') as f:
            f.writelines(lines)