# This message is used to add items to the EMR by the user

# REQUIRED 
# Type of Container being transmitted
#
# Accepted types and meanings: 
#   MAP - a MapContainer.msg object
#   OBJECT - an ObjectContainer.msg object
#   COMPONTENT - a ComponentContainer.msg object
#   ROBOT - a RobotContainer.msg object
string type

string maintainer

# REQUIRED
# Contains serialized container
uint8[] serialized_container