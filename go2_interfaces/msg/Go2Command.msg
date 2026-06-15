std_msgs/Header header
uint8 STAND=0
uint8 MOVE=1
uint8 TRICK=2

uint8 command_type
geometry_msgs/Twist twist_command # Used if MOVE
string trick_name     # Used if TRICK