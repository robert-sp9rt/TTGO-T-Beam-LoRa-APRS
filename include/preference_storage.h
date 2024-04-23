#include <Preferences.h>

#ifndef PREF_STORAGE
#define PREF_STORAGE

#define ENABLE_PREFERENCES
extern Preferences preferences;

// MAX 15 chars for preference key!!!
static const char *const PREF_AP_PASSWORD = "ap_password";
static const char *const PREF_WIFI_SSID = "wifi_ssid";
static const char *const PREF_WIFI_PASSWORD = "wifi_password";
static const char *const PREF_NTP_SERVER = "ntp_server";
static const char *const PREF_SYSLOG_SERVER = "syslog_server";
static const char *const PREF_WIFI_ENABLE_INIT = "wifi_en_i";
static const char *const PREF_WIFI_ENABLE = "wifi_en";
static const char *const PREF_WIFI_TXPWR_MODE_AP_INIT = "wifi_pwrAP_i";
static const char *const PREF_WIFI_TXPWR_MODE_AP = "wifi_pwrAP";
static const char *const PREF_WIFI_TXPWR_MODE_STA_INIT = "wifi_pwrSTA_i";
static const char *const PREF_WIFI_TXPWR_MODE_STA = "wifi_pwrSTA";
static const char *const PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED_INIT = "wifi_failAP_i";
static const char *const PREF_WIFI_STA_ALLOW_FAILBACK_TO_MODE_AP_AFTER_ONCE_CONNECTED = "wifi_failAP";
static const char *const PREF_TNCSERVER_ENABLE_INIT = "tncsrvr_en_i";
static const char *const PREF_TNCSERVER_ENABLE = "tncsrvr_en";
static const char *const PREF_GPSSERVER_ENABLE_INIT = "gpssrv_en_i";
static const char *const PREF_GPSSERVER_ENABLE = "gpssrv_en";


// LoRa settings
static const char *const PREF_LORA_FREQ_PRESET_INIT = "lora_freq_i";
static const char *const PREF_LORA_FREQ_PRESET = "lora_freq";
static const char *const PREF_LORA_SPEED_PRESET_INIT = "lora_speed_i";
static const char *const PREF_LORA_SPEED_PRESET = "lora_speed";
static const char *const PREF_LORA_RX_ENABLE_INIT = "lora_rx_en_i";
static const char *const PREF_LORA_RX_ENABLE = "lora_rx_en";
static const char *const PREF_LORA_TX_ENABLE_INIT = "lora_tx_en_i";
static const char *const PREF_LORA_TX_ENABLE = "lora_tx_en";
static const char *const PREF_LORA_TX_POWER_INIT = "txPower_i";
static const char *const PREF_LORA_TX_POWER = "txPower";
static const char *const PREF_LORA_TX_PREAMBLE_LEN_INIT = "preambleLen_i";
static const char *const PREF_LORA_TX_PREAMBLE_LEN = "preambleLen";
static const char *const PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET_INIT = "lora_rssi2p_i";
static const char *const PREF_LORA_ADD_SNR_RSSI_TO_PATH_PRESET = "lora_rssi2p";
static const char *const PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET_INIT = "snraprsis_i";
static const char *const PREF_LORA_ADD_SNR_RSSI_TO_PATH_END_AT_KISS_PRESET = "snraprsis";
static const char *const PREF_LORA_FREQ_CROSSDIGI_PRESET_INIT = "lora_freq_x_i";
static const char *const PREF_LORA_FREQ_CROSSDIGI_PRESET = "lora_freq_x";
static const char *const PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET_INIT = "tx_qrg_bc_i";
static const char *const PREF_LORA_TX_BEACON_AND_KISS_TO_FREQUENCIES_PRESET = "tx_qrg_bc";
static const char *const PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET_INIT = "tx_aprsis_bc_i";
static const char *const PREF_LORA_TX_BEACON_AND_KISS_TO_APRSIS_PRESET = "tx_aprsis_bc";
static const char *const PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET_INIT = "tx_aprsis_sm_i";
static const char *const PREF_LORA_TX_STATUSMESSAGE_TO_APRSIS_PRESET = "tx_aprsis_sm";
static const char *const PREF_LORA_SPEED_CROSSDIGI_PRESET_INIT = "lora_speed_x_i";
static const char *const PREF_LORA_SPEED_CROSSDIGI_PRESET = "lora_speed_x";
static const char *const PREF_LORA_TX_POWER_CROSSDIGI_PRESET_INIT = "txPower_x_i";
static const char *const PREF_LORA_TX_POWER_CROSSDIGI_PRESET = "txPower_x";
static const char *const PREF_LORA_RX_ON_FREQUENCIES_PRESET_INIT = "rx_qrg_i";
static const char *const PREF_LORA_RX_ON_FREQUENCIES_PRESET = "rx_qrg";
static const char *const PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET_INIT = "lora_cradapt_i";
static const char *const PREF_LORA_AUTOMATIC_CR_ADAPTION_PRESET = "lora_cradapt";
static const char *const PREF_APRS_DIGIPEATING_MODE_PRESET_INIT = "lora_dig_mode_i";
static const char *const PREF_APRS_DIGIPEATING_MODE_PRESET = "lora_dig_mode";
static const char *const PREF_APRS_DIGIPEATING_MYALIAS_INIT = "lora_myalias_i";
static const char *const PREF_APRS_DIGIPEATING_MYALIAS = "lora_myalias";
static const char *const PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET_INIT = "lora_dig_x_m_i";
static const char *const PREF_APRS_CROSS_DIGIPEATING_MODE_PRESET = "lora_dig_x_m";

// Station settings
static const char *const PREF_APRS_CALLSIGN = "aprs_callsign";
static const char *const PREF_APRS_RELAY_PATH = "aprs_relay_path";
static const char *const PREF_APRS_RELAY_PATH_INIT = "aprs_relay_init";
static const char *const PREF_APRS_SYMBOL_TABLE = "aprs_s_table";
static const char *const PREF_APRS_SYMBOL = "aprs_symbol";
static const char *const PREF_APRS_OBJECT_NAME = "aprs_objnam";
static const char *const PREF_APRS_OBJECT_NAME_INIT = "aprs_objnam_i";
static const char *const PREF_APRS_COMMENT = "aprs_comment";
static const char *const PREF_APRS_COMMENT_INIT = "aprs_comm_init";
static const char *const PREF_APRS_COMMENT_RATELIMIT_PRESET = "aprs_comm_rt";
static const char *const PREF_APRS_COMMENT_RATELIMIT_PRESET_INIT = "aprs_comm_rt_i";
static const char *const PREF_APRS_SHOW_ALTITUDE = "aprs_alt";			/* obsoleted. may be removed later. Use alt ratio 0 .. 100*/
static const char *const PREF_APRS_SHOW_ALTITUDE_INIT = "aprs_alt_init";	/* obsoleted. may be removed later. Use alt ratio 0 .. 100*/
static const char *const PREF_APRS_ALTITUDE_RATIO = "aprs_alt_r";
static const char *const PREF_APRS_ALTITUDE_RATIO_INIT = "aprs_alt_r_i";
static const char *const PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE = "aprs_CSandA";
static const char *const PREF_APRS_ALWAYS_SEND_CSE_SPEED_AND_ALTITUDE_INIT = "aprs_CSandA_i";

static const char *const PREF_APRS_SHOW_BATTERY = "aprs_batt";
static const char *const PREF_APRS_SHOW_BATTERY_INIT = "aprs_batt_init";
static const char *const PREF_APRS_STATUS_WINLINK_NOTIFICATION = "aprs_stat_wl";
static const char *const PREF_APRS_STATUS_WINLINK_NOTIFICATION_INIT = "aprs_stat_wl_i";
static const char *const PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION = "aprs_stat_qrt";
static const char *const PREF_APRS_STATUS_SHUTDOWN_NOTIFICATION_INIT = "aprs_stat_qrt_i";
static const char *const PREF_APRS_LATITUDE_PRESET = "aprs_lat_p";
static const char *const PREF_APRS_LATITUDE_PRESET_INIT = "aprs_lat_p_init";
static const char *const PREF_APRS_LATLON_FROM_GPS = "aprs_llfgps"; // no _INIT needed. Checkbox is evaluated on "save" in the Web interface
static const char *const PREF_APRS_POSITION_AMBIGUITY = "pos_amb";
static const char *const PREF_APRS_POSITION_AMBIGUITY_INIT = "pos_amb_init";
static const char *const PREF_APRS_LONGITUDE_PRESET = "aprs_lon_p";
static const char *const PREF_APRS_LONGITUDE_PRESET_INIT = "aprs_lon_p_init";
static const char *const PREF_APRS_SENDER_BLACKLIST = "aprs_blist";
static const char *const PREF_APRS_SENDER_BLACKLIST_INIT = "aprs_blist_init";
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
static const char *const PREF_TNC_SELF_TELEMETRY_ALLOW_RF = "tnc_tel_rf";
static const char *const PREF_TNC_SELF_TELEMETRY_ALLOW_RF_INIT = "tnc_tel_rf_i";

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
static const char *const PREF_DEV_LOGTOSERIAL_EN = "logserial_en";		/* obsoleted. may be removed later. Use alt ratio 0 .. 100*/
static const char *const PREF_DEV_LOGTOSERIAL_EN_INIT = "logserial_en_i";	/* obsoleted. may be removed later. Use alt ratio 0 .. 100*/
static const char *const PREF_DEV_USBSERIAL_DATA_TYPE = "usbser_proto";
static const char *const PREF_DEV_USBSERIAL_DATA_TYPE_INIT= "usbser_proto_i";
static const char *const PREF_DEV_OL_EN = "oled_enabled";
static const char *const PREF_DEV_OL_EN_INIT = "ol_enabled_init";
static const char *const PREF_DEV_SHOW_RX_TIME = "sh_rxtime";
static const char *const PREF_DEV_SHOW_RX_TIME_INIT = "sh_rxtime_init";
static const char *const PREF_DEV_AUTO_SHUT = "shutdown_act";
static const char *const PREF_DEV_AUTO_SHUT_INIT = "shutdown_actini";
static const char *const PREF_DEV_AUTO_SHUT_PRESET = "shutdown_dt";
static const char *const PREF_DEV_AUTO_SHUT_PRESET_INIT = "shutdown_dtini";
static const char *const PREF_DEV_REBOOT_INTERVAL = "reboot_intrvl";
static const char *const PREF_DEV_REBOOT_INTERVAL_INIT = "reboot_intrvl_i";
static const char *const PREF_DEV_SHOW_OLED_TIME = "sh_oledtime"; // set OLED timeout
static const char *const PREF_DEV_SHOW_OLED_TIME_INIT = "sh_oledtime_i";
static const char *const PREF_DEV_CPU_FREQ = "cpufreq";
static const char *const PREF_DEV_CPU_FREQ_INIT = "cpufreq_i";
static const char *const PREF_DEV_UNITS = "units";
static const char *const PREF_DEV_UNITS_INIT = "units_i";
static const char *const PREF_DEV_OLED_L3_L4_FORMAT = "oledl3l4fmt";
static const char *const PREF_DEV_OLED_L3_L4_FORMAT_INIT = "oledl3l4fmt_i";
static const char *const PREF_DEV_OLED_LOCATOR = "oledlocator";
static const char *const PREF_DEV_OLED_LOCATOR_INIT = "locator_i";
static const char *const PREF_DEV_OLED_LOCATOR_AMBIGUITY = "oled_loc_amb";
static const char *const PREF_DEV_OLED_LOCATOR_AMBIGUITY_INIT = "oled_loc_amb_i";


// APRSIS settings
static const char *const PREF_APRSIS_EN_INIT = "aprsis_en_i";
static const char *const PREF_APRSIS_EN = "aprsis_en";
static const char *const PREF_APRSIS_SERVER_NAME_INIT = "aprsis_srv_h_i";
static const char *const PREF_APRSIS_SERVER_NAME = "aprsis_srv_h";
static const char *const PREF_APRSIS_SERVER_PORT_INIT = "aprsis_srv_p_i";
static const char *const PREF_APRSIS_SERVER_PORT = "aprsis_srv_p";
static const char *const PREF_APRSIS_FILTER_INIT = "aprsis_fltr_i";
static const char *const PREF_APRSIS_FILTER = "aprsis_fltr";
static const char *const PREF_APRSIS_FILTER_LOCAL_INCOMING_INIT = "aprsis_fli_i";
static const char *const PREF_APRSIS_FILTER_LOCAL_INCOMING = "aprsis_fli";
static const char *const PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING_INIT = "aprsis_fliw_i";
static const char *const PREF_APRSIS_FILTER_LOCAL_WORDS_INCOMING = "aprsis_fliw";
static const char *const PREF_APRSIS_CALLSIGN_INIT = "aprsis_call_i";
static const char *const PREF_APRSIS_CALLSIGN = "aprsis_call";
static const char *const PREF_APRSIS_PASSWORD_INIT = "aprsis_pw_i";
static const char *const PREF_APRSIS_PASSWORD = "aprsis_pw";
static const char *const PREF_APRSIS_ALLOW_INET_TO_RF_INIT = "aprsis_2rf_i";
static const char *const PREF_APRSIS_ALLOW_INET_TO_RF = "aprsis_2rf";

#endif
