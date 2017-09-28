#ifndef __NSPO_GPS_H__
#define __NSPO_GPS_H__
/********************************* TRICK HEADER *******************************
PURPOSE:
      (NSPO GPS Receiver interface definition)
*******************************************************************************/
#include <stdint.h>

struct bitfield_16_t {
    uint32_t _0  : 1;
    uint32_t _1  : 1;
    uint32_t _2  : 1;
    uint32_t _3  : 1;
    uint32_t _4  : 1;
    uint32_t _5  : 1;
    uint32_t _6  : 1;
    uint32_t _7  : 1;
    uint32_t _8  : 1;
    uint32_t _9  : 1;
    uint32_t _10 : 1;
    uint32_t _11 : 1;
    uint32_t _12 : 1;
    uint32_t _13 : 1;
    uint32_t _14 : 1;
    uint32_t _15 : 1;
} __attribute__((packed));

struct bitfield_32_t {
    uint32_t _0  : 1;
    uint32_t _1  : 1;
    uint32_t _2  : 1;
    uint32_t _3  : 1;
    uint32_t _4  : 1;
    uint32_t _5  : 1;
    uint32_t _6  : 1;
    uint32_t _7  : 1;
    uint32_t _8  : 1;
    uint32_t _9  : 1;
    uint32_t _10 : 1;
    uint32_t _11 : 1;
    uint32_t _12 : 1;
    uint32_t _13 : 1;
    uint32_t _14 : 1;
    uint32_t _15 : 1;
    uint32_t _16 : 1;
    uint32_t _17 : 1;
    uint32_t _18 : 1;
    uint32_t _19 : 1;
    uint32_t _20 : 1;
    uint32_t _21 : 1;
    uint32_t _22 : 1;
    uint32_t _23 : 1;
    uint32_t _24 : 1;
    uint32_t _25 : 1;
    uint32_t _26 : 1;
    uint32_t _27 : 1;
    uint32_t _28 : 1;
    uint32_t _29 : 1;
    uint32_t _30 : 1;
    uint32_t _31 : 1;
} __attribute__((packed));


struct float_xyz_t {
    float x;
    float y;
    float z;
} __attribute__((packed));

struct ECEF_navigation_and_status_data_t {
    uint16_t        GPS_week_number;                /* *io  (--)    GPS week number since Epoch */
    uint32_t        GPS_time;                       /* *io  (ms)    GPS time tag from start of the week */
    uint16_t        validity;                       /* *io  (--)    Navigation data vaild? 0:invalid, 1:valid */
    uint16_t        PDOP;                           /* *io  (one-2) Positon PDOP; Max 32767 */
    struct float_xyz_t     pos;                            /* *io  (m)     Host vehicle position in ECEF frame */
    struct float_xyz_t     vel;                            /* *io  (m/s)   Host vehicle velocity in ECEF frame */
    int32_t         oscillator_offset;              /* *io  (--)    Device oscillator offset */
    int64_t         clock_bias;                     /* *io  (s-12)  Device clock bias */
    struct bitfield_32_t   health_of_satellites_map;       /* *io  (--)    If bit n set, (n + 1) satellite is healthy */
    struct bitfield_32_t   visibility_of_satellites_map;   /* *io  (--)    If bit n set, (n + 1) satellite is visible */
} __attribute__((packed));

struct configuration_setup_message_t {
    int16_t         elevation_mask;                 /* *io  (d)     Satellite elevation mask (-90 ~ +90) */
    uint8_t         RAIM_threshold;                 /* *io  (m) */
    uint8_t         C_NO_threshold_for_tracking;    /* *io  (dB) */
} __attribute__((packed));

struct time_and_channel_status_data_t {
    uint8_t         SV_ID;                          /* *io  (--)    SV ID */
    uint8_t         SNR;                            /* *io  (dB*Hz) SNR */
    int16_t         azimuth;                        /* *io  (d)     Azimuth (0 ~ 360) */
    int16_t         elevation;                      /* *io  (d)     Elevation (-90 ~ 90) */
    uint32_t        delay;                          /* *io  (m)     pseudo delay (0 ~ 30,000,000) */
    int32_t         doopler;                        /* *io  (--)    Doppler  (-60000 ~ 60000)*/
    uint16_t        phase;                          /* *io  (d)     carrier phase (0 ~ 360) */
    uint16_t        position;                       /* *io  (--)    code position (0 ~ 8184) */
    float           time_shift;                     /* *io  (s)     time shift */
    struct float_xyz_t     pos;                            /* *io  (m)     position */
    struct float_xyz_t     vel;                            /* *io  (m/s)   velocity */
} __attribute__((packed));

struct ECEF_OP_navigation_and_status_t {
    struct bitfield_16_t   nav_status;                     /* *io  (--)    See Technical Manual */
} __attribute__((packed));

struct GPS_UTC_clock_correction_parameter_t {
    uint16_t        delta_t_LS;                     /* *io  (s)     Delta time due to leap seconds */
} __attribute__((packed));

struct NSPO_GPSR_SCI_TLM_t {
    struct ECEF_navigation_and_status_data_t       ECEF_navigation_and_status_data;
    struct configuration_setup_message_t           configuration_setup_message;
    struct time_and_channel_status_data_t          channel_1;
    struct time_and_channel_status_data_t          channel_2;
    struct time_and_channel_status_data_t          channel_3;
    struct time_and_channel_status_data_t          channel_4;
    struct time_and_channel_status_data_t          channel_5;
    struct time_and_channel_status_data_t          channel_6;
    struct time_and_channel_status_data_t          channel_7;
    struct time_and_channel_status_data_t          channel_8;
    struct time_and_channel_status_data_t          channel_9;
    struct time_and_channel_status_data_t          channel_10;
    struct time_and_channel_status_data_t          channel_11;
    struct time_and_channel_status_data_t          channel_12;
    struct ECEF_OP_navigation_and_status_t         ECEF_OP_navigation_and_status;
    struct GPS_UTC_clock_correction_parameter_t    GPS_UTC_clock_correction_parameter;
    float                                   reserved_0;
    float                                   reserved_1;
    uint8_t				    reserved[2];
} __attribute__((packed));

#endif  // __NSPO_GPS__
