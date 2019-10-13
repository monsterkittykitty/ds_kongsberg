# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

# High-level info
string sounder_name
string ship_name
bool kctrl_connected    # KCtrl --> node
bool pu_powered         # PU --> KCtrl
bool pu_connected       # PU <-- KCtrl
bool kmall_connected    # PU --> node
bool pinging
bool commanded_pinging

# BIST
bool bist_running
string bist_current
uint8 bist_progress
uint16 bist_count
string bist_filename
string bist_directory

# Data info
uint8 kmall_filecount
string kmall_filename
string kmall_directory
uint64 kmall_filesize_kB
uint16 xml_filecount
string xml_filename
string xml_directory

# Health and sensors
# uint8 SENSOR_UNKNOWN = 0
# uint8 SENSOR_OK = 1
# uint8 SENSOR_MISSING = 2
# uint8 SENSOR_OFF = 3
# uint8 SENSOR_ACTIVE_BAD = 4
# uint8 SENSOR_BAD = 5

string cpu_temperature
string position_1
string position_2
string position_3
string attitude_1
string attitude_2
string depth
string svp
string time_sync
string pps
bool sensors_all_good
# bool sensors_any_missing
# bool sensors_any_bad

# Runtime params
# depth settings
string rt_force_depth #
string rt_min_depth #
string rt_max_depth #
string rt_ping_freq #
string rt_depth_mode #
string rt_detector_mode #
string rt_fm_disable #
string rt_log_watercol
string rt_extra_detect #

# txmit params
string rt_pitch_stab #
string rt_tx_angle_along #
string rt_yaw_stab #
# string rt_heading_filter # VALUE NOT FOUND, WILL BE EMPTY
string rt_max_ping_rate #
string rt_min_swath_distance #
string rt_trigger #

# Latest ping
uint16 ping_num
uint16 num_soundings
uint8 percent_good
uint16 max_depth
uint16 min_depth
uint16 center_depth
uint16 max_range
uint16 min_range
uint16 center_range
