// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define FRAME_CONFIG QUAD_FRAME
#define FRAME_ORIENTATION X_FRAME

#define CH7_OPTION        AUX_SWITCH_DO_NOTHING

#define USERHOOK_VARIABLES "UserVariables.h"
#define USERHOOK_INIT userhook_init();                    // for code to be run once at startup
#define USERHOOK_FASTLOOP userhook_FastLoop();            // for code to be run at 100hz
#define USERHOOK_50HZLOOP userhook_50Hz();                // for code to be run at 50hz
#define USERHOOK_MEDIUMLOOP userhook_MediumLoop();        // for code to be run at 10hz
//#define USERHOOK_SUPERSLOWLOOP userhook_SuperSlowLoop();  // for code to be run at 1hz

#define AC_FENCE DISABLED

// ----------------

#define COURIERDRONE_GNSS_AUTOREPORT 1
#define COURIERDRONE_MANUAL_THRUST_YAW 0

#define CRDR_AUTOPILOT_INPUT_TIMEOUT 400

// 250000 baud can be managed precisely, the closest standard value 230400 baud is 7.8% off
#define SERIAL0_BAUD 115200
// baudrate of serial3 used for serial0 when USB is disconnected; WTF?
#define SERIAL3_BAUD SERIAL0_BAUD

// Emergency manual control:
#define MANUAL_CONTROL_HYSTERESIS         20
#define MANUAL_CONTROL_SELECTOR_CHANNEL   6
#define MANUAL_CONTROL_SELECTOR_THRESHOLD 1800
#define MANUAL_CONTROL_THRUSTEN_CHANNEL   7
#define MANUAL_CONTROL_THRUSTEN_THRESHOLD 1800
