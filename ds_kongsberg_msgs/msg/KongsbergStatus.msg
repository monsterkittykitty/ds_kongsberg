# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

uint8 STATUS_GOOD = 1
uint8 STATUS_BAD = 2
uint8 STATUS_NOTCONNECTED

uint8 status

uint8 pinging # 1 yes, 0 no

uint8 powered # 1 yes, 0 no

