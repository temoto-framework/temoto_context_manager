# REQUIRED - Name of the robot
string name

# REQUIRED - How the location of this robot can be acquired
string[] detection_methods

# Name of EMR parent item
string parent

string odom_frame_id
string base_frame_id

geometry_msgs/PoseStamped pose