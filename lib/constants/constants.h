#pragma once
#include <stdint.h>

namespace Constants
{

    //--------------------Pines for arduino mega--------------------

    namespace Motor
    {
        namespace Left
        {
            const uint8_t in1 = 26;
            const uint8_t in2 = 27;
            const uint8_t pwm = 4;
            constexpr float speedOffset = 5.0f;
        }

        namespace Center
        {
            const uint8_t in1 = 24;
            const uint8_t in2 = 25;
            const uint8_t pwm = 5;
            constexpr float speedOffset = 0.0f;
        }

        namespace Right
        {
            const uint8_t in1 = 22;
            const uint8_t in2 = 23;
            const uint8_t pwm = 7;
            constexpr float speedOffset = 0.0f;
        }

        const double minPWM = 40.0;  // We set the minimum PWM that the robot needs to move
        const double maxPWM = 255.0; // We set a max PWM to limit the motors movement for better performance and control
    }

    //--------------------Pins for multiplexer and phototransistors--------------------

    const uint8_t kSignalPin1 = A6;
    const uint8_t kMUXPin1_1 = 17;
    const uint8_t kMUXPin2_1 = 28;
    const uint8_t kMUXPin3_1 = 29;

    const uint8_t kSignalPin2 = A7;
    const uint8_t kMUXPin1_2 = 43;
    const uint8_t kMUXPin2_2 = 44;
    const uint8_t kMUXPin3_2 = 45;

    const uint8_t kSignalPin3 = A8;
    const uint8_t kMUXPin1_3 = 14;
    const uint8_t kMUXPin2_3 = 15;
    const uint8_t kMUXPin3_3 = 16;

    // Phototransistors
    const uint8_t kPhotoElements = 8; // Number of phototransistor channels per side

    // Side-wide thresholds kept for compatibility with older tests/debug prints.
    const int kPhotoTresholdLeft = 380;
    const int kPhotoTresholdRight = 300;
    const int kPhotoTresholdFront = 200;

    // Per-channel thresholds for each sensor independently.
    // Phototransitor delta thresholds for line detection
    const uint16_t kPhotoMargins[3][kPhotoElements] = {
        {50, 50, 50, 50, 50, 50, 50, 50},
        {50, 50, 50, 50, 50, 50, 50, 50},
        {40, 40, 40, 40, 40, 40, 40, 40}
    };

    const unsigned long kAvoidDurationMs = 350;
    const uint8_t kBaselineSamples = 20;
    const uint16_t kBaselineDelayMs = 10;

    // -----------Ultrasonic sensor--------------
    const uint8_t kTrigPin = 33;
    const uint8_t kEchoPin = 32;

    const uint8_t kMinGoalKeeperTresholdY = 35; // Minimum distance to the goal in cm
    const uint8_t kMaxGoalKeeperTresholdY = 65; // Maximum distance to the goal in cm
    const int kLeftGoalKeeperTresholdX = 116;   // Minimum distance to the goal in cm
    const int kRightGoalKeeperTresholdX = 275;  // Maximum distance to the goal in cm
    const uint8_t kGoalKeeperTresholdX = 15;

    // ----------------- IMU -------------------
    const float kIMUDeadbandThreshold = 0.5f; // degrees per second
    const double kIMUMaxDt = 0.1; // 100 ms
}