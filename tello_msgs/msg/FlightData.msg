# ============================================================
# |                    Fligh data msg type.                  |
# | Contains all the information that is sent from the drone |
# ============================================================

# =========================================================================

# - Battery data
int16 battery_low = 0
int16 battery_lower = 0
int16 battery_percentage = 0
int16 drone_battery_left = 0
int16 drone_fly_time_left = 0

# =========================================================================

# - States
int16 battery_state = 0
int16 camera_state = 0
int16 electrical_machinery_state = 0
int16 down_visual_state = 0
int16 gravity_state = 0
int16 imu_calibration_state = 0
int16 imu_state = 0
int16 power_state = 0
int16 pressure_state = 0
int16 wind_state = 0

# =========================================================================

# - Stats
int16 drone_hover = 0
int16 em_open = 0
int16 em_sky = 0
int16 em_ground = 0
int16 factory_mode = 0
int16 fly_mode = 0
int16 fly_time = 0
int16 front_in = 0
int16 front_lsc = 0
int16 front_out = 0

# =========================================================================

# - Sensors
int16 fly_speed = 0
int16 east_speed = 0
int16 ground_speed = 0
int16 height = 0
int16 light_strength = 0
int16 north_speed = 0
int16 temperature_height = 0

# =========================================================================

# - Other
int16 outage_recording = 0
int16 smart_video_exit_mode = 0
int16 throw_fly_timer = 0

# =========================================================================

# - WiFi
int16 wifi_disturb = 0
int16 wifi_strength = 0
