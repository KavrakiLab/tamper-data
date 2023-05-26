#! /usr/bin/env python

import rospy
import yaml
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from std_msgs.msg import String


# package_dir = "package://onto/resources/real_robot/sense_tamp"
benchmark_dirs = {"h3" : "/horizontal_stack_3obj",
                  "ds" : "/drawer_shelf"}

def read_YAML_data(file_name):
    '''Returns the yaml data structure of the data stored.
    '''
    with open(file_name) as input_file:
        return yaml.load(input_file.read())

class Object:
    ''' A collision object with its model and pose
    '''
    def __init__(self, name, yaml_file, pose):
        # yaml_file has to be absolute path
        self.name = name
        self.pose = pose

        data = read_YAML_data(yaml_file)
        self.vicon_name = data["name"]
        self.frame = data["frame"]
        self.type = data["geometry"]["type"]
        if self.type == "mesh":
            self.resource = data["geometry"]["resource"]
        else:
            self.dimensions = data["geometry"]["dimensions"]

def load_objects(problem_file):
    # problem_file has to be absolute path
    print("problem_file", problem_file)
    data = read_YAML_data(problem_file)
    objects = []
    for obj in data.items():
        if obj[0] == "joints":#TODO
            continue
        pose = Pose()
        pose.position.x = obj[1]["pose"]["position"]["x"]
        pose.position.y = obj[1]["pose"]["position"]["y"]
        pose.position.z = obj[1]["pose"]["position"]["z"]
        pose.orientation.x = obj[1]["pose"]["orientation"]["x"]
        pose.orientation.y = obj[1]["pose"]["orientation"]["y"]
        pose.orientation.z = obj[1]["pose"]["orientation"]["z"]
        pose.orientation.w = obj[1]["pose"]["orientation"]["w"]
        collision_object = Object(obj[0], obj[1]["vicon_model_path"], pose)
        objects.append(collision_object)
    return objects

def publish_objects(objects):
    publisher = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=2)

    markers = MarkerArray()

    m_id = 0
    for obj in objects:
        marker = Marker()
        marker.header.frame_id = "/base_link" # For Fetch robot
        if obj.type == "box":
            marker.type = marker.CUBE
            marker.scale.x = obj.dimensions[0]
            marker.scale.y = obj.dimensions[1]
            marker.scale.z = obj.dimensions[2]
        elif obj.type == "mesh":
            marker.type = marker.MESH_RESOURCE
            marker.mesh_resource = "file://" + obj.resource
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
        marker.action = marker.ADD
        marker.color.a = 0.8
        marker.color.r = 0.9
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.pose = obj.pose
        marker.id = m_id
        markers.markers.append(marker)
        m_id += 1

    print("publishing...")
    while not rospy.is_shutdown():
        publisher.publish(markers)
        rospy.rostime.wallsleep(1.0)

def main():
    rospy.init_node('tamp_benchmark_marker')

    print("Please specify the problem class that you want to record poses. Options are:")
    print("    h3     :   horizontal stacking 3obj")
    print("    ds     :   drawer & shelf")
    problem_class = raw_input("Please type your choice here: ")
    # bench_dir = resolve_path(package_dir + "/benchmarks" + benchmark_dirs[problem_class])
    bench_dir = "/home/robot/catkin_ws/src/dataset" + benchmark_dirs[problem_class]

    problem_id = raw_input("Please type the problem id here: ")

    # problem_file = resolve_path("{}/{}.yaml".format(bench_dir, problem_id))
    problem_file = "{}/{}.yaml".format(bench_dir, problem_id)
    publish_objects(load_objects(problem_file))

if __name__ == '__main__':
    main()
