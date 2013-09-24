/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    25  // called at 1hz so 25 seconds

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
static void arm_motors_check()
{
    if (!motors.armed() == !crdr_armed)
        return;

    if (crdr_armed)
    {
        pre_arm_checks(true);
        if (!ap.pre_arm_check)
            return;
        init_arm_motors();
        gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMED"));
    }
    else
    {
        init_disarm_motors();
        gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMED"));
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 25 seconds
// called at 1hz
static void auto_disarm_check()
{
    // No need to autodisarm anything
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
static void init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
//     static bool did_ground_start = false;

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    hal.uartA->set_blocking_writes(false);
    if (gcs3.initialised) {
        hal.uartC->set_blocking_writes(false);
    }

#if COPTER_LEDS == ENABLED
    piezo_beep_twice();
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    // Reset home position
    // -------------------
    if(ap.home_is_set)
        init_home();

    // all I terms are invalid
    // -----------------------
    reset_I_all();

//     if(did_ground_start == false) {
//         did_ground_start = true;
//         startup_ground();
//     }

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground -
    // this resets Baro for more accuracy
    //-----------------------------------
//     init_barometer();
#endif

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(false);
#endif

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);

    // set hover throttle
    motors.set_mid_throttle(g.throttle_mid);

#if COPTER_LEDS == ENABLED
    piezo_beep_twice();
#endif

    // Cancel arming if throttle is raised too high so that copter does not suddenly take off
    read_radio();
    if (g.rc_3.control_in > g.throttle_cruise && g.throttle_cruise > 100) {
        motors.output_min();
        failsafe_enable();
        return;
    }

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // enable output to motors
    output_min();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // reenable failsafe
    failsafe_enable();
}

// perform pre-arm checks and set ap.pre_arm_check flag
static void pre_arm_checks(bool display_failure)
{
    // exit immediately if we've already successfully performed the pre-arm check
    if( ap.pre_arm_check ) {
        return;
    }

    // succeed if pre arm checks are disabled
    if(!g.arming_check_enabled) {
        set_pre_arm_check(true);
        return;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return;
    }
    
    // pre-arm check to ensure ch7 and ch8 have different functions
    if ((g.ch7_option != 0 || g.ch8_option != 0) && g.ch7_option == g.ch8_option) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Ch7&Ch8 Opt cannot be same"));
        }
        return;
    }

    // check accelerometers have been calibrated
    if(!ins.calibrated()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
        }
        return;
    }

    // check the compass is healthy
    if(!compass.healthy) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
        }
        return;
    }

    // check compass learning is on or offsets have been set
    Vector3f offsets = compass.get_offsets();
    if(!compass._learn && offsets.length() == 0) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
        }
        return;
    }

    // check for unreasonable compass offsets
    if(offsets.length() > 500) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
        }
        return;
    }

    // check for unreasonable mag field length
    float mag_field = pythagorous3(compass.mag_x, compass.mag_y, compass.mag_z);
    if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
        }
        return;
    }

    // barometer health check
    if(!barometer.healthy) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Baro not healthy"));
        }
        return;
    }

#if AC_FENCE == ENABLED
    // check fence is initialised
    if(!fence.pre_arm_check() || (((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) && g_gps->hdop > g.gps_hdop_good)) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Bad GPS Pos"));
        }
        return;
    }
#endif

#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check board voltage
    if(board_voltage() < BOARD_VOLTAGE_MIN || board_voltage() > BOARD_VOLTAGE_MAX) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Board Voltage"));
        }
        return;
    }
#endif

    // failsafe parameter checks
    if (g.failsafe_throttle) {
        // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
        if (g.rc_3.radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
            }
            return;
        }
    }

    // lean angle parameter check
    if (g.angle_max < 1000 || g.angle_max > 8000) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
        }
        return;
    }

    // pass arming checks at least once
    if (!arm_checks(display_failure)) {
        return;
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
static void pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // check if radio has been calibrated
    if(!g.rc_3.radio_min.load()) {
        return;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (g.rc_1.radio_min > 1300 || g.rc_1.radio_max < 1700 || g.rc_2.radio_min > 1300 || g.rc_2.radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (g.rc_3.radio_min > 1300 || g.rc_3.radio_max < 1700 || g.rc_4.radio_min > 1300 || g.rc_4.radio_max < 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    ap.pre_arm_rc_check = true;
}

// arm_checks - perform final checks before arming
// always called just before arming.  Return true if ok to arm
static bool arm_checks(bool display_failure)
{
    // succeed if arming checks are disabled
    if(!g.arming_check_enabled) {
        return true;
    }

    // check gps is ok if required
    if(mode_requires_GPS(control_mode) && (!GPS_ok() || g_gps->hdop > g.gps_hdop_good)) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Bad GPS Pos"));
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

static void init_disarm_motors()
{
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);

    compass.save_offsets();

    g.throttle_cruise.save();

    // we are not in the air
    set_takeoff_complete(false);

#if COPTER_LEDS == ENABLED
    piezo_beep();
#endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(true);
#endif

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void
set_servos_4()
{
#if FRAME_CONFIG == TRI_FRAME
    // To-Do: implement improved stability patch for tri so that we do not need to limit throttle input to motors
    g.rc_3.servo_out = min(g.rc_3.servo_out, 800);
#endif
    motors.output();
}

