#pragma once
#include <stdint.h>

namespace Constants
{

    //--------------------Robot pin map (validate against Teensy 4.1 wiring)--------------------

    namespace Motor
    {
        namespace Left
        {
            const uint8_t pwm = 2;
            const uint8_t in1 = 3;
            const uint8_t in2 = 4;
            constexpr float speedOffset = 0.0f;
            constexpr float directionSign = 1.0f;
        }

        namespace Center
        {
            const uint8_t pwm = 5;    
            const uint8_t in1 = 6;
            const uint8_t in2 = 7;
            constexpr float speedOffset = 0.0f;
            constexpr float directionSign = -1.0f;
        }

        namespace Right
        {
            const uint8_t pwm = 8;
            const uint8_t in1 = 9;
            const uint8_t in2 = 10;
            constexpr float speedOffset = 0.0f;
            constexpr float directionSign = -1.0f;
        }

        const double minPWM = 40.0;  // We set the minimum PWM that the robot needs to move
        const double maxPWM = 255.0; // We set a max PWM to limit the motors movement for better performance and control
    }

    //--------------------Pins for multiplexer and phototransistors--------------------

    // These names match the physical robot sides.
    // The left/right boards are wired opposite of the old bank numbering, so keep
    // the side labels here authoritative and use them everywhere else.
    const uint8_t kPhotoLeftSignalPin = 17;
    const uint8_t kPhotoLeftMuxS0Pin = 38;
    const uint8_t kPhotoLeftMuxS1Pin = 39;
    const uint8_t kPhotoLeftMuxS2Pin = 40;

    // Keep right/front matched to the current physical mux routing on the robot.
    const uint8_t kPhotoRightSignalPin = 21;
    const uint8_t kPhotoRightMuxS0Pin = 34;
    const uint8_t kPhotoRightMuxS1Pin = 35;
    const uint8_t kPhotoRightMuxS2Pin = 36;

    const uint8_t kPhotoFrontSignalPin = 16;
    const uint8_t kPhotoFrontMuxS0Pin = 30;
    const uint8_t kPhotoFrontMuxS1Pin = 31;
    const uint8_t kPhotoFrontMuxS2Pin = 32;

    // Phototransistors
    const uint8_t kPhotoLedEnablePin = 20; // Drive HIGH to power the floor illumination LEDs
    const uint8_t kPhotoElements = 8; // Number of phototransistor channels per side
    // Debugging for enabling/disabling specific phototransistor sides without rewiring 
    constexpr bool kPhotoLeftEnabled = false;
    constexpr bool kPhotoRightEnabled = true;
    constexpr bool kPhotoFrontEnabled = true;

    // Side-wide thresholds kept for compatibility with older tests/debug prints.
    const int kPhotoTresholdLeft = 380;
    const int kPhotoTresholdRight = 300;
    const int kPhotoTresholdFront = 200;

    constexpr uint16_t kPhotoIgnoredMargin = 4095;

    // Per-channel thresholds for each sensor independently.
    // Left channels 2, 4, and 6 are intentionally ignored by giving them a
    // margin larger than any normal phototransistor delta.
    const uint16_t kPhotoMargins[3][kPhotoElements] = {
{125, 175, kPhotoIgnoredMargin, 175, kPhotoIgnoredMargin, 175, kPhotoIgnoredMargin, 175},
{45, 45, 45, 45, 45, 45, 45, 45},
 {55, 55, 55, 55, 55, 55, 55, 55}
    };

    const unsigned long kAvoidDurationMs = 450;
    const uint8_t kBaselineSamples = 20;
    const uint16_t kBaselineDelayMs = 10;
    constexpr unsigned long kBaselineSettleDelayMs = 2000;
    const unsigned long kPhotoAutoBaselineQuietMs = 3000;

    // Goalie home-goal window. Tune these after mounting or tilting the Pixy.
    const uint8_t kMinGoalKeeperTresholdY = 35;
    const uint8_t kMaxGoalKeeperTresholdY = 65;
    const int kLeftGoalKeeperTresholdX = 116;
    const int kRightGoalKeeperTresholdX = 275;
    const uint8_t kGoalKeeperTresholdX = 15;

    // ----------------- IMU -------------------
    const float kIMUDeadbandThreshold = 0.5f; // degrees per second
    const double kIMUMaxDt = 0.1; // 100 ms
    constexpr uint32_t kIRSerialBaud = 9600;
    constexpr unsigned long kIRFreshDataTimeoutMs = 200;

    namespace Striker
    {
        // Main ball-chase heading hold used by Colibri and PIDLookForBall-style chase.
        constexpr float kHeadingKp = 2.5f;
        constexpr float kHeadingKd = 0.15f;
        constexpr float kMaxTurnPwm = 55.0f;
        constexpr float kMinTurnPwm = 35.0f;
        constexpr float kHeadingSettleBandDeg = 6.5f;

        constexpr float kChaseDrivePwmRatio = 0.52f;
        constexpr float kGoalDrivePwmRatio = 0.55f;
        constexpr float kAvoidDrivePwmRatio = 0.75f;
        constexpr unsigned long kStartupHoldMs = 500;

        // Heading hold used specifically during avoid-line maneuvers.
        constexpr float kAvoidHeadingKp = 1.5f;
        constexpr float kAvoidHeadingKd = 0.10f;
        constexpr float kAvoidMaxTurnPwm = 55.0f;
        constexpr float kAvoidMinTurnPwm = 50.0f;
        constexpr float kAvoidHeadingSettleBandDeg = 6.0f;

        // The ball is considered "in front" inside this +- angle window.
        constexpr float kBallFrontToleranceDeg = 6.0f;
        // When the ball is off-center but still in the front half, ease in behind it.
        constexpr float kBehindBallApproachGain = 0.6f;
        constexpr float kBehindBallApproachClampDeg = 55.0f;
        // When the ball is behind the robot, strafe around it instead of hitting backward.
        constexpr float kBehindBallOrbitAngleDeg = 0.85f;

        constexpr float kIRBallFollowOffsetBack = 1.0f;
        constexpr float kIRBallFollowOffsetSide = 1.0f;
        constexpr float kIRBallFollowOffsetFront = 1.0f;
        constexpr float kIRFarBallStrength = 4.0f;
        constexpr float kIRCloseBallStrength = 10.0f;
        // Tune this if the striker cuts too sharply while chasing the ball.
        constexpr float kIRBallAngleClampDeg = 18.0f;
        constexpr float kIRBallDetectedStrength = 3.0f;
        constexpr float kIRPossessionStrength = 10.0f;
        // Main striker aiming threshold when using the angle-only UART ball feed.
        constexpr float kIRPossessionAngleToleranceDeg = 24.0f;

        // Retune after moving the camera or retraining goal signatures.
        constexpr unsigned long kGoalCaptureTimeoutMs = 1500;
        constexpr unsigned long kGoalLostTimeoutMs = 450;
        constexpr float kGoalAngleClampDeg = 8.0f;
        constexpr float kGoalHeadingDeadbandDeg = 2.5f;
        constexpr float kAngleSmoothingAlpha = 0.18f;
        constexpr uint16_t kGoalCenterTolerancePx = 17;
        // Increase if the robot shoots too early, decrease if it waits too long to aim.
        constexpr uint32_t kGoalAimAreaThreshold = 2500;
    }

    namespace Goalie
    {
        constexpr float kHeadingKp = 2.3f;
        constexpr float kHeadingKd = 0.18f;
        constexpr float kMaxTurnPwm = 90.0f;
        constexpr float kMinTurnPwm = 24.0f;
        constexpr float kHeadingSettleBandDeg = 5.0f;

        constexpr float kHomeDrivePwmRatio = 0.30f;
        constexpr float kDefenseDrivePwmRatio = 0.26f;
        constexpr float kInterceptDrivePwmRatio = 0.38f;
        constexpr float kRetreatDrivePwmRatio = 0.28f;

        constexpr unsigned long kRetreatDurationMs = 1200;
        constexpr unsigned long kGoalCaptureTimeoutMs = 1500;
        constexpr unsigned long kGoalLostTimeoutMs = 450;

        // Flip this sign if the backward-facing Pixy correction strafes the wrong way.
        constexpr float kGoalTrackDirectionSign = 1.0f;
        // Tune these three first if the goalie drifts left/right while centered on goal.
        constexpr float kGoalAngleClampDeg = 40.0f;
        constexpr float kGoalCenterDeadbandDeg = 5.0f;
        constexpr float kGoalCorrectionWeight = 0.35f;

        // Tune these if the goalie reacts too late or over-slides on ball tracking.
        constexpr float kBallAngleClampDeg = 55.0f;
        constexpr float kBallFollowWeight = 0.65f;
        constexpr float kInterceptDeadbandDeg = 4.0f;
        constexpr float kGoalSearchDrivePwmRatio = 0.22f;
        constexpr float kIRCloseBallStrength = 6.0f;
        constexpr float kIRThreatAngleToleranceDeg = 95.0f;
    }

    namespace Ajolote
    {
        // Ajolote copies the striker flow, but at lower speed for tighter control.
        constexpr float kChaseDrivePwmRatio = 0.44f;
        constexpr float kAvoidDrivePwmRatio = 0.58f;
        constexpr float kHomeRecoveryDrivePwmRatio = 0.32f;
        constexpr float kGoalSearchDrivePwmRatio = 0.22f;
        constexpr unsigned long kStartupHoldMs = 1500;

        // Tune this first during field testing so the robot only roams as far as you want.
        constexpr uint32_t kHomeGoalMinAreaThreshold = 1800;
        constexpr unsigned long kGoalCaptureTimeoutMs = 1500;
        constexpr unsigned long kGoalLostTimeoutMs = 450;
        constexpr float kHomeGoalAngleClampDeg = 40.0f;
        constexpr float kHomeGoalCenterDeadbandDeg = 5.0f;
    }
}
