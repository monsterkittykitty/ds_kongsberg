

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

uint16 TYPE_EMPTY=0
uint16 TYPE_STATUS_IPI=12
uint16 TYPE_READY=39
uint16 TYPE_STATUS_IPU=616
uint16 TYPE_FILEIMPORTED=476 # 451,476
uint16 TYPE_PARAMSREAD=459
uint16 TYPE_VALUES=401

uint16 type
string[] payload