#include <Preferences.h>

#ifndef PREF_STORAGE
#define PREF_STORAGE

#define ENABLE_PREFERENCES
extern Preferences preferences;

// MAX 15 chars for preference key!!!
static const char *const PREF_WIFI_SSID = "wifi_ssid";
static const char *const PREF_WIFI_PASSWORD = "wifi_password";
static const char *const PREF_AP_PASSWORD = "ap_password";
// LoRa settings
static const char *const PREF_LORA_FREQ_PRESET_INIT = "lora_freq_i";
static const char *const PREF_LORA_FREQ_PRESET = "lora_freq";
static const char *const PREF_LORA_SPEED_PRESET_INIT = "lora_speed_i";
static const char *const PREF_LORA_SPEED_PRESET = "lora_speed";
static const char *const PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT = "lora_rssi2p_i";
static const char *const PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET = "lora_rssi2p";
static const char *const PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT = "lora_freq_x_i";
static const char *const PREF_LORA_FREQ_CROSSDIGI_PRESET = "lora_freq_x";
static const char *const PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT = "lora_speed_x_i";
static const char *const PREF_LORA_SPEED_CROSSDIGI_PRESET = "lora_speed_x";
static const char *const PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT = "lora_cradapt_i";
static const char *const PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET = "lora_cradapt";
static const char *const PREF_APRS_DIGIPEATING_MODE_PRESET_INIT = "lora_dig_mode_i";
static const char *const PREF_APRS_DIGIPEATING_MODE_PRESET = "lora_dig_mode";
static const char *const PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT = "lora_dig_x_m_i";
static const char *const PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET = "lora_dig_x_m";

// Station settings
static const char *const PREF_APRS_CALLSIGN = "aprs_callsign";
static const char *const PREF_APRS_RELAY_PATH = "aprs_relay_path";
static const char *const PREF_APRS_RELAY_PATH_INIT = "aprs_relay_init";
static const char *const PREF_APRS_SYMBOL_TABLE = "aprs_s_table";
static const char *const PREF_APRS_SYMBOL = "aprs_symbol";
static const char *const PREF_APRS_COMMENT = "aprs_comment";
static const char *const PREF_APRS_COMMENT_INIT = "aprs_comm_init";
static const char *const PREF_APRS_COMMENT_RATELIMIT_PRESET = "aprs_comm_rt_i";
static const char *const PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT = "aprs_comm_rt";
static const char *const PREF_APRS_SHOW_ALTITUDE = "aprs_alt";
static const char *const PREF_APRS_SHOW_ALTITUDE_INIT = "aprs_alt_init";
static const char *const PREF_APRS_SHOW_BATTERY = "aprs_batt";
static const char *const PREF_APRS_SHOW_BATTERY_INIT = "aprs_batt_init";
static const char *const PREF_APRS_LATITUDE_PRESET = "aprs_lat_p";
static const char *const PREF_APRS_LATITUDE_PRESET_INIT = "aprs_lat_p_init";
static const char *const PREF_APRS_LONGITUDE_PRESET = "aprs_lon_p";
static const char *const PREF_APRS_LONGITUDE_PRESET_INIT = "aprs_lon_p_init";
static const char *const PREF_APRS_FIXED_BEACON_PRESET = "aprs_fixed_beac";
static const char *const PREF_APRS_FIXED_BEACON_PRESET_INIT = "aprs_fix_b_init";
static const char *const PREF_APRS_FIXED_BEACON_INTERVAL_PRESET = "aprs_fb_interv";
static const char *const PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT = "aprs_fb_in_init";
static const char *const PREF_ENABLE_TNC_SELF_TELEMETRY = "tnc_tel";
static const char *const PREF_ENABLE_TNC_SELF_TELEMETRY_INIT = "tnc_tel_i";
static const char *const PREF_TNC_SELF_TELEMETRY_INTERVAL = "tnc_tel_int";
static const char *const PREF_TNC_SELF_TELEMETRY_INTERVAL_INIT = "tnc_tel_int_i";
static const char *const PREF_TNC_SELF_TELEMETRY_SEQ = "tnc_tel_seq";
static const char *const PREF_TNC_SELF_TELEMETRY_SEQ_INIT = "tnc_tel_seq_i";
static const char *const PREF_TNC_SELF_TELEMETRY_MIC = "tnc_tel_mic";
static const char *const PREF_TNC_SELF_TELEMETRY_MIC_INIT = "tnc_tel_mic_i";
static const char *const PREF_TNC_SELF_TELEMETRY_PATH = "tnc_tel_path";
static const char *const PREF_TNC_SELF_TELEMETRY_PATH_INIT = "tnc_tel_path_i";

// SMART BEACONING
static const char *const PREF_APRS_SB_MIN_INTERVAL_PRESET = "sb_min_interv";
static const char *const PREF_APRS_SB_MIN_INTERVAL_PRESET_INIT = "sb_min_interv_i";
static const char *const PREF_APRS_SB_MAX_INTERVAL_PRESET = "sb_max_interv";
static const char *const PREF_APRS_SB_MAX_INTERVAL_PRESET_INIT = "sb_max_interv_i";
static const char *const PREF_APRS_SB_MIN_SPEED_PRESET = "sb_min_speed";
static const char *const PREF_APRS_SB_MIN_SPEED_PRESET_INIT = "sb_min_speed_i";
static const char *const PREF_APRS_SB_MAX_SPEED_PRESET = "sb_max_speed";
static const char *const PREF_APRS_SB_MAX_SPEED_PRESET_INIT = "sb_max_speed_i";
static const char *const PREF_APRS_SB_ANGLE_PRESET = "sb_angle";
static const char *const PREF_APRS_SB_ANGLE_PRESET_INIT = "sb_angle_i";
static const char *const PREF_APRS_SB_TURN_SLOPE_PRESET = "sb_turn_slope";
static const char *const PREF_APRS_SB_TURN_SLOPE_PRESET_INIT = "sb_turn_slope_i";
static const char *const PREF_APRS_SB_TURN_TIME_PRESET = "sb_turn_time";
static const char *const PREF_APRS_SB_TURN_TIME_PRESET_INIT = "sb_turn_time_i";

// Device settings
static const char *const PREF_APRS_GPS_EN = "gps_enabled";
static const char *const PREF_APRS_GPS_EN_INIT = "gps_state_init";
static const char *const PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS = "kiss_myloc_ok";
static const char *const PREF_ACCEPT_OWN_POSITION_REPORTS_VIA_KISS_INIT = "kiss_myloc_ok_i";
static const char *const PREF_GPS_ALLOW_SLEEP_WHILE_KISS = "gps_sleep_ok";
static const char *const PREF_GPS_ALLOW_SLEEP_WHILE_KISS_INIT = "gps_sleep_ok_i";
static const char *const PREF_APRS_SHOW_CMT = "show_cmt";
static const char *const PREF_APRS_SHOW_CMT_INIT = "show_cmt_init";
static const char *const PREF_DEV_BT_EN = "bt_enabled";
static const char *const PREF_DEV_BT_EN_INIT = "bt_enabled_init";
static const char *const PREF_DEV_OL_EN = "oled_enabled";
static const char *const PREF_DEV_OL_EN_INIT = "ol_enabled_init";
static const char *const PREF_DEV_SHOW_RX_TIME = "sh_rxtime";
static const char *const PREF_DEV_SHOW_RX_TIME_INIT = "sh_rxtime_init";
static const char *const PREF_DEV_AUTO_SHUT = "shutdown_act";
static const char *const PREF_DEV_AUTO_SHUT_INIT = "shutdown_actini";
static const char *const PREF_DEV_AUTO_SHUT_PRESET = "shutdown_dt";
static const char *const PREF_DEV_AUTO_SHUT_PRESET_INIT = "shutdown_dtini";
static const char *const PREF_DEV_SHOW_OLED_TIME = "sh_oledtime"; // set OLED timeout
static const char *const PREF_DEV_SHOW_OLED_TIME_INIT = "sh_oledtime_i";

#endif
