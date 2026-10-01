# defines a MediaPipe detection
int32 id

# Handedness: "Left" or "Right"
string handedness
float32 score

# 21 Image-space landmarks (x, y, z) 
# x and y are normalized [0.0, 1.0] by image width/height
# z represents depth with the origin at the wrist
geometry_msgs/Point[] landmarks

# 21 World landmarks (x, y, z)
# Coordinates in meters with the origin at the hand's geometric center
geometry_msgs/Point[] world_landmarks