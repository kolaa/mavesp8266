
#ifndef MAVOSD_H
#define MAVOSD_H

#define MAH_CALIBRATION_FACTOR                                      1.0f    //used to calibrate mAh reading.
#define SPEED_IN_KILOMETERS_PER_HOUR                                        //if commented out defaults to m/s
//#define IMPERIAL_UNITS                                                    //Altitude in feet, distance to home in miles.
#define VEHICLE_TYPE                                                1       //0==ArduPlane, 1==ArduCopter, 2==INAVPlane, 3==INAVCopter. Used for flight modes
#define STORE_GPS_LOCATION_IN_SUBTITLE_FILE                                 //comment out to disable. Stores GPS location in the goggles .srt file in place of the "uavBat:" field at a slow rate of ~2-3s per GPS coordinate
//#define DISPLAY_THROTTLE_POSITION                                         //will display the current throttle position(0-100%) in place of the osd_roll_pids_pos element.
//#define DISPLAY_WIND_SPEED_AND_DIRECTION                                  //Ardupilot only


#include <Arduino.h>
#include "ardupilotmega/version.h"
#include "mavlink_types.h"

/// MAVLink system definition
extern mavlink_system_t mavlink_system;
static inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {}
static inline uint8_t comm_receive_ch(mavlink_channel_t chan) { return 0; }
static inline uint16_t comm_get_available(mavlink_channel_t chan) { return 0; }
static inline int comm_get_txspace(mavlink_channel_t chan){ return -1; }

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#include "ardupilotmega/mavlink.h"

#include "MSP.h"
#include "MSP_OSD.h"
#include "flt_modes.h"
#include "OSD_positions_config.h"
#include <SoftwareSerial.h>

class MavOSD {
public:
    MavOSD();

    void    begin           ();
    void    sendToOSD       (mavlink_message_t* msg, mavlink_status_t* status);
    uint32_t    packets_sent = 0;

private:
    void    check_system_status();
    void    show_text(char (*text)[15]);
    void    blink_sats();
    void    mAh_drawn_calc();
    void    display_wind_speed_and_direction();
    void    set_flight_mode_flags();
    void    set_battery_cells_number();
    void    send_osd_config();
    void    send_msp_to_airunit();
    void    display_flight_mode();
    void    mavl_receive(mavlink_message_t* msg, mavlink_status_t* status);
    void    GPS_calc_longitude_scaling(int32_t lat);
    void    GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing);
    void    GPS_calculateDistanceAndDirectionToHome(void);

private:    
    SoftwareSerial mspSerial;
    MSP msp;

    uint32_t previousMillis_MSP = 0;
    const uint32_t next_interval_MSP = 100;

    uint8_t base_mode = MAV_MODE_PREFLIGHT;
    uint8_t system_status = MAV_STATE_UNINIT;
    uint32_t custom_mode = 0;                       //flight mode

    uint8_t vbat = 0;
    float airspeed = 0;
    float groundspeed = 0;
    int32_t relative_alt = 0;       // in milimeters
    uint32_t altitude_msp = 0;      // EstimatedAltitudeCm
    uint16_t rssi = 0;
    uint8_t battery_remaining = 0;
    uint32_t flightModeFlags = 0;
    char craftname[15];
    int16_t amperage = 0;
    uint16_t mAhDrawn = 0;
    float f_mAhDrawn = 0.0;
    uint8_t numSat = 1;
    uint8_t pid_roll[3];
    uint8_t pid_pitch[3];
    uint8_t pid_yaw[3];
    int32_t gps_lon = 0;
    int32_t gps_lat = 0;
    int32_t gps_alt = 0;
    int32_t gps_home_lon = 0;
    int32_t gps_home_lat = 0;
    int32_t gps_home_alt = 0;
    int16_t roll_angle = 0;
    int16_t pitch_angle = 0;
    uint32_t distanceToHome = 0;    // distance to home in meters
    int16_t directionToHome = 0;   // direction to home in degrees
    uint8_t fix_type = 0;           // < 0-1: no fix, 2: 2D fix, 3: 3D fix
    uint8_t batteryCellCount = 0;
    uint16_t batteryCapacity = 3000;
    uint8_t legacyBatteryVoltage = 0;
    uint8_t batteryState = 0;       // voltage color 0==white, 1==red
    uint16_t batteryVoltage = 0;
    int16_t heading = 0;
    float dt = 0;
    #ifdef MAH_CALIBRATION_FACTOR
    float mAh_calib_factor = MAH_CALIBRATION_FACTOR;
    #else
    float mAh_calib_factor = 1;
    #endif
    uint8_t set_home = 1;
    uint32_t general_counter = 0;
    uint16_t _osd_gps_sats_pos = osd_gps_sats_pos;
    uint16_t blink_sats_orig_pos = osd_gps_sats_pos;
    uint16_t blink_sats_blank_pos = 234;
    uint32_t previousFlightMode = custom_mode;
    uint8_t srtCounter = 1;
    uint8_t thr_position = 0;
    float wind_direction = 0;   // wind direction (degrees)
    float wind_speed = 0;       // wind speed in ground plane (m/s)
    float relative_wind_direction = 0;
    float climb_rate = 0.0;

    msp_osd_config_t msp_osd_config = {0};    
    msp_battery_state_t battery_state = {0};
    msp_name_t name;
    //msp_fc_version_t fc_version = {0};
    msp_status_BF_t status_BF = {0};
    msp_analog_t analog = {0};
    msp_raw_gps_t raw_gps = {0};
    msp_comp_gps_t comp_gps = {0};
    msp_attitude_t attitude = {0};
    msp_altitude_t altitude = {0};
    #ifdef DISPLAY_THROTTLE_POSITION
    msp_pid_t pid = {0};
    #endif

    float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles
};


#endif
