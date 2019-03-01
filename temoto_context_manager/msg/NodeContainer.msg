# This message is used to add nodes to the EMR by the user

# REQUIRED 
# Type of Container being transmitted
#
# Accepted types and meanings: 
#   MAP - a MapContainer.msg object
#   OBJECT - an ObjectContainer.msg object
string type

# REQUIRED
# Contains serialized ObjectContainer or MapContainer
uint8[] serialized_container