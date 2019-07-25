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

# BIST
bool bist_running
uint8 bist_progress
uint16 bist_count
string bist_filename
string bist_directory

# Data info
uint8 kmall_filecount
string kmall_filename
string kmall_directory
uint16 kmall_filesize_kB
uint16 xml_filecount
string xml_filename
string xml_directory

# Health and sensors
uint8 SENSOR_UNKNOWN = 0
uint8 SENSOR_OK = 1
uint8 SENSOR_MISSING = 2
uint8 SENSOR_OFF = 3
uint8 SENSOR_ACTIVE_BAD = 4
uint8 SENSOR_BAD = 5

uint8 cpu_temperature
uint8 position_1
uint8 position_2
uint8 position_3
uint8 attitude_1
uint8 attitude_2
uint8 depth
uint8 svp
uint8 time_sync
uint8 pps
bool sensors_all_good
bool sensors_any_missing
bool sensors_any_bad

# Runtime params
# depth settings
float32 rt_force_depth
uint8 rt_min_depth
uint8 rt_max_depth
uint16 rt_ping_freq
string rt_depth_mode
string rt_detector_mode
bool rt_fm_disable
bool rt_log_watercol
bool rt_extra_detect

# txmit params
bool rt_pitch_stab
float32 rt_tx_angle_along
string rt_yaw_stab
string rt_heading_filter
float32 rt_max_ping_rate
uint16 rt_min_swath_distance
bool rt_trigger

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