# Hand Kinematic retargeting implementing 'DexPilot: Vision Based Teleoperation of Dexterous Robotic Hand-Arm System' (2019)
1. Start custom mediapipe-docker to run hand tracking
~~~bash
docker compose run --rm yolopipe
~~~
2. Start a camera acquisition (e.g)
~~~bash
ros2 run usb_cam usb_cam_node_exe
~~~
3. Start a kinematic retargeting
~~~bash
ros2 run allegro_hand_teleoperation kinematic_retargeting.py
~~~

The kinematic retargeting node will publish the following topics:
- `allegroHand_0/joint_cmd` - JointStates (For hand controlling)
- `retargeting_debug_markers` - MarkerArray (For Rviz visualization purpouse)