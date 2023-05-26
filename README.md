# Build the image
```
docker build . -t tamper/replicate-ros-bag
```

# Run the pipeline to play the ros bag
### Step 1: Launch Roscore
Start the docker container and launch the roscore
```
./run.bash
roscore
```

### Step 2: Publish Fetch Head Camera TF
Open another terminal window. Get the ID of this running docker container by `docker ps`. Find the line that has the docker image name tamper/replicate-ros-bag, and manually copy its ID (example: 0b3e9f753363). Then, run the following to enter the same docker container
```
docker exec -it 0b3e9f753363 bash
```

We need to publish the transform from head camera frame to one of the robot links, to make sure the recorded point cloud and RGB images can show up in RViz correctly
```
<!-- rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 head_pan_link head_camera_rgb_optical_frame -->
rosrun tf2_ros static_transform_publisher 0.176 0.021 0.079 -0.481 0.479 -0.524 0.515 head_pan_link head_camera_rgb_optical_frame
- Translation: [0.139, 0.052, 1.119]
- Rotation: in Quaternion [-0.523, 0.432, -0.475, 0.561]

```

### Step 3: Play the Rosbag
Open the third terminal window, and enter the docker container as described above. Then, you can play any of the recorded rosbags (e.g., 30_result.bag) by 
```
rosbag play src/dataset/30_result.bag
```
For more information on playing around with rosbags, please see the official instructions by ROS: ...

Open up RViz in another terminal to visualize the recorded real-time camera input. To visualize the point cloud, add the topic ... To visualize the RGB image, add the topic ...

### Step 4: Visualize in RViz
This helps you visualize the setting of each experiment (e.g., ground truth of the initial poses of the objects). You need the TFs of the robot in place to let RViz know where the objects should be in the workspace. If you are running this on your own robot platform, you probably already have that. Otherwise, you can play any of the provided rosbags, and hit space to toggle pause the rosbag, just to publish some kind of TFs. Then, visualize an experiment setting by entering the container in the 4th terminal, and
```
cd src/scripts
./benchmark_publish.py 
```

Follow the prompts to visualize the experiments of all the benchmarks. You can open up RViz in another terminal, and add the topic `/visualization_marker_array`. After adding the topic, if the markers are not immediately showing up, you may want to resume replaying and then pause the rosbag to hav
