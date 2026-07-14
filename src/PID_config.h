#ifndef _PID_CONFIG_H_
#define _PID_CONFIG_H_

namespace Config {
    // Loop Frequency / Time delta
    constexpr float DT            = 0.001f;  // 1kHz loop rate (1/1000s) for Inner Rate Loop
    constexpr float DT_SLOW       = 0.005f;  // 200Hz loop rate (1/200s) for Outer Angle Loop
    constexpr float FILTER_CUTOFF = 50.0f;   // 50Hz low-pass filter cutoff

    // Actuator Limits (Inner Loop Motor Adjustments)
    constexpr float MAX_LIMIT     = 10000.0f;  // change this to a smaller value when using motors
    constexpr float MIN_LIMIT     = -10000.0f; 

    // Command Limits (Outer Loop Target Rate Commands)
    constexpr float MAX_RATE_LIMIT = 400.0f; // Max target rate of 200 deg/s
    constexpr float MIN_RATE_LIMIT = -400.0f;

    // Roll Rate PID Tuning Constants (Inner)
    constexpr float KP_ROLL_RATE  = 3.5f;
    constexpr float KI_ROLL_RATE  = 0.00f;
    constexpr float KD_ROLL_RATE  = 0.00f;

    // Pitch Rate PID Tuning Constants (Inner)
    constexpr float KP_PITCH_RATE = 3.5f;
    constexpr float KI_PITCH_RATE = 0.00f;
    constexpr float KD_PITCH_RATE = 0.00f;
    
    // Yaw Rate PID Tuning Constants (Inner)
    constexpr float KP_YAW_RATE   = 2.0f;
    constexpr float KI_YAW_RATE   = 0.02f;
    constexpr float KD_YAW_RATE   = 0.0f;

    // Angle PID Tuning Constants (Outer)
    constexpr float KP_ROLL_ANGLE  = 8.0f;    
    constexpr float KP_PITCH_ANGLE = 8.0f;
}

#endif