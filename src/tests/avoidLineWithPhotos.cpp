#include <Arduino.h>
#include "photo.h"
#include "constants.h"
#include "PID.h"
#include "robot.h"

Robot robot;
Phototransistor phototransistor_sensors(
    Constants::kPhotoLeftSignalPin, Constants::kPhotoLeftMuxS0Pin, Constants::kPhotoLeftMuxS1Pin, Constants::kPhotoLeftMuxS2Pin,
    Constants::kPhotoRightSignalPin, Constants::kPhotoRightMuxS0Pin, Constants::kPhotoRightMuxS1Pin, Constants::kPhotoRightMuxS2Pin,
    Constants::kPhotoFrontSignalPin, Constants::kPhotoFrontMuxS0Pin, Constants::kPhotoFrontMuxS1Pin, Constants::kPhotoFrontMuxS2Pin
);

const float drivePwm = 0.55f * Constants::Motor::maxPWM;
const float kHeadingKp = 1.5f;
const float kHeadingKd = 0.10f;
const float kMaxTurnPwm = 65.0f;
const float kMinTurnPwm = 40.0f;
const float kHeadingSettleBandDeg = 5.5f;
constexpr unsigned long kLateralMuxSwitchIntervalMs = 150;
constexpr uint8_t kLiveBaselineSamples = 8;
constexpr uint16_t kLiveBaselineDelayMs = 5;
enum class RobotState { IDLE, AVOIDING_LINE };
RobotState current_state = RobotState::IDLE;

PID headingPD(kHeadingKp, 0.0f, kHeadingKd, kMaxTurnPwm, kMinTurnPwm, kHeadingSettleBandDeg);
double targetYaw = 0.0;
unsigned long avoid_start_time = 0;
int escapeAngle = 0;
unsigned long last_debug_time = 0;
unsigned long last_lateral_mux_switch_ms = 0;
Side active_lateral_side = Side::Left;

void printPhotoPinMap()
{
    Serial.println("Photo pin map:");

    Serial.print("  LED enable: ");
    Serial.println(Constants::kPhotoLedEnablePin);

    Serial.print("  Left signal/select: ");
    Serial.print(Constants::kPhotoLeftSignalPin);
    Serial.print(" / ");
    Serial.print(Constants::kPhotoLeftMuxS0Pin);
    Serial.print(", ");
    Serial.print(Constants::kPhotoLeftMuxS1Pin);
    Serial.print(", ");
    Serial.println(Constants::kPhotoLeftMuxS2Pin);

    Serial.print("  Right signal/select: ");
    Serial.print(Constants::kPhotoRightSignalPin);
    Serial.print(" / ");
    Serial.print(Constants::kPhotoRightMuxS0Pin);
    Serial.print(", ");
    Serial.print(Constants::kPhotoRightMuxS1Pin);
    Serial.print(", ");
    Serial.println(Constants::kPhotoRightMuxS2Pin);

    Serial.print("  Front signal/select: ");
    Serial.print(Constants::kPhotoFrontSignalPin);
    Serial.print(" / ");
    Serial.print(Constants::kPhotoFrontMuxS0Pin);
    Serial.print(", ");
    Serial.print(Constants::kPhotoFrontMuxS1Pin);
    Serial.print(", ");
    Serial.println(Constants::kPhotoFrontMuxS2Pin);
}

void printTriggeredLineSides()
{
    if (phototransistor_sensors.HasLineOnSide(Side::Left))
    {
        Serial.println("LINE on LEFT");
    }
    if (phototransistor_sensors.HasLineOnSide(Side::Right))
    {
        Serial.println("LINE on RIGHT");
    }
    if (phototransistor_sensors.HasLineOnSide(Side::Front))
    {
        Serial.println("LINE on FRONT");
    }
}

const char *sideName(Side side)
{
    switch (side)
    {
    case Side::Right:
        return "RIGHT";
    case Side::Left:
    default:
        return "LEFT";
    }
}

void retakeActiveLateralBaseline()
{
    phototransistor_sensors.CaptureSideBaseline(
        active_lateral_side,
        kLiveBaselineSamples,
        kLiveBaselineDelayMs);
    Serial.print("Retook ");
    Serial.print(sideName(active_lateral_side));
    Serial.println(" baseline");
}

void primeLateralMuxSelection()
{
    phototransistor_sensors.AdvanceLateralMuxCycle();
    active_lateral_side = Side::Left;
    retakeActiveLateralBaseline();
    last_lateral_mux_switch_ms = millis();
}

void updateLateralMuxSelection(unsigned long nowMs)
{
    if ((nowMs - last_lateral_mux_switch_ms) >= kLateralMuxSwitchIntervalMs)
    {
        last_lateral_mux_switch_ms = nowMs;
        phototransistor_sensors.AdvanceLateralMuxCycle();
        active_lateral_side = (active_lateral_side == Side::Left) ? Side::Right : Side::Left;
        retakeActiveLateralBaseline();
    }
}

void setup()
{
    Serial.begin(115200);
#if defined(CORE_TEENSY)
    analogReadResolution(12);
    analogReadAveraging(2);
#endif

    robot.begin();
    robot.motors.stop();
    delay(2000);
    targetYaw = robot.bno.GetBNOData();
    headingPD.reset();

    phototransistor_sensors.Initialize();
    phototransistor_sensors.SetAllMargins(Constants::kPhotoMargins);
    phototransistor_sensors.CaptureBaseline(Constants::kBaselineSamples, Constants::kBaselineDelayMs);
    phototransistor_sensors.SetAlternatingLateralMuxEnabled(true);
    primeLateralMuxSelection();
    printPhotoPinMap();
    Serial.println("avoidLineWithPhotos ready");
}

void loop()
{
    updateLateralMuxSelection(millis());
    const double yaw = robot.bno.GetBNOData();
    const double turnCommand = headingPD.calculate(targetYaw, yaw, true);

    if (current_state == RobotState::AVOIDING_LINE)
    {
        if (millis() - avoid_start_time >= Constants::kAvoidDurationMs)
        {
            current_state = RobotState::IDLE;
        }
        else
        {
            robot.motors.move(escapeAngle, drivePwm, turnCommand);
        }
        return;
    }

    if (millis() - last_debug_time >= 120)
    {
        last_debug_time = millis();
        phototransistor_sensors.PhotoDebug();
        if (phototransistor_sensors.HasLineOnSide(Side::Left))
        {
            Serial.println("LINE on LEFT");
        }
        if (phototransistor_sensors.HasLineOnSide(Side::Right))
        {
            Serial.println("LINE on RIGHT");
        }
        if (phototransistor_sensors.HasLineOnSide(Side::Front))
        {
            Serial.println("LINE on FRONT");
        }
        Serial.println();
    }
    
    int escapeAngleDetected = phototransistor_sensors.CheckPhotosOnField();
    if (escapeAngleDetected != -1)
    {
        escapeAngle = escapeAngleDetected;
        avoid_start_time = millis();
        current_state = RobotState::AVOIDING_LINE;
        printTriggeredLineSides();
        Serial.print("Line detected, escaping at ");
        Serial.println(escapeAngle);
        robot.motors.move(escapeAngle, drivePwm, turnCommand);
        return;
    }

    robot.motors.move(0.0f, 0.0f, turnCommand);
}
