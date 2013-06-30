/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * Courierdrone patch
 * Pavel Kirienko, 2012
 */

static const int NUM_CHANNELS = 8;

bool crdr_armed = false;
bool crdr_manual = false;

static long unsigned int crdr_last_autopilot_input = 0;

enum ControlMode
{
    CONTROL_MODE_NONE,
    CONTROL_MODE_MANUAL,
    CONTROL_MODE_AUTOPILOT
};

static int crdr_last_control_mode = CONTROL_MODE_NONE;

void crdr_reset_autopilot_input_timeout()
{
    crdr_last_autopilot_input = millis();
}

int crdr_get_current_control_mode()
{
    if (crdr_manual)
        return CONTROL_MODE_MANUAL;
    if (crdr_last_autopilot_input && ((crdr_last_autopilot_input + CRDR_AUTOPILOT_INPUT_TIMEOUT) > millis()))
        return CONTROL_MODE_AUTOPILOT;
    return CONTROL_MODE_NONE;
}

void crdr_set_default_rc_override()
{
    int16_t v[NUM_CHANNELS] =
    {
        1500, // roll (or pitch?)
        1500, // pitch (or roll?)
        900,  // thrust (off)
        1500, // yaw
        0, 0, 0, 0   // no overriding
    };
    hal.rcin->set_overrides(v, NUM_CHANNELS);
    crdr_manual = false;
}

void userhook_init()
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Courierdrone patch "__DATE__" "__TIME__));
    crdr_set_default_rc_override();
}

class ManualControlChannel
{
    const char channel_;
    const int threshold_;
    bool active_;

public:
    ManualControlChannel(char channel, int threshold)
    : channel_(channel)
    , threshold_(threshold)
    , active_(false)
    { }

    bool Poll()
    {
        uint16_t ppm = hal.rcin->read(channel_ - 1);
        if (active_)
            ppm += MANUAL_CONTROL_HYSTERESIS;
        else
            ppm -= MANUAL_CONTROL_HYSTERESIS;
        bool new_active = ppm > threshold_;
        return active_ = new_active;
    }
};

ManualControlChannel mcchan_selector(MANUAL_CONTROL_SELECTOR_CHANNEL, MANUAL_CONTROL_SELECTOR_THRESHOLD);
ManualControlChannel mcchan_thrusten(MANUAL_CONTROL_THRUSTEN_CHANNEL, MANUAL_CONTROL_THRUSTEN_THRESHOLD);

static void update_mode()
{
    const bool manual_control_enabled = mcchan_selector.Poll();
    if (manual_control_enabled)
    {
        hal.rcin->clear_overrides();
        crdr_manual = true;
    }
    else
        crdr_manual = false;

    // thrust arming
    crdr_armed = mcchan_thrusten.Poll();

    // control lock
    const int cm = crdr_get_current_control_mode();
    if (cm == CONTROL_MODE_NONE)
        crdr_set_default_rc_override();
    if (cm != crdr_last_control_mode)
    {
        crdr_last_control_mode = cm;
        gcs_send_text_fmt(PSTR("CONTROL MODE %i"), int(cm));
    }
}

static void update_leds()
{
    switch (crdr_get_current_control_mode())
    {
    case CONTROL_MODE_MANUAL:
        digitalWrite(A_LED_PIN, LED_ON);
        digitalWrite(B_LED_PIN, LED_OFF);
        digitalWrite(C_LED_PIN, LED_OFF);
        break;
    case CONTROL_MODE_AUTOPILOT:
        digitalWrite(A_LED_PIN, LED_OFF);
        digitalWrite(B_LED_PIN, LED_OFF);
        digitalWrite(C_LED_PIN, LED_ON);
        break;
    default:
        digitalWrite(A_LED_PIN, LED_OFF);
        digitalWrite(B_LED_PIN, LED_ON);
        digitalWrite(C_LED_PIN, LED_OFF);
        break;
    }
}

void userhook_FastLoop()
{
    gcs_send_message(MSG_RAW_IMU1);  // raw IMU data
    gcs_send_message(MSG_ATTITUDE);  // computed attitude and angular rates
    // host syncronization is based on the MSG_ATTITUDE message, so it must be sent in the last order
}

void userhook_50Hz()
{
    gcs_send_message(MSG_RAW_IMU2);  // air pressure and temperature
    update_mode();
    update_leds();
}

void userhook_MediumLoop()
{
    // TODO: send power state
}
