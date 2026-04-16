#pragma once

#include "Pixy.h"
#include "Pixy2.h"
#include "TPixy2.h"
#include "Pixy2UART.h"
#include "Pixy2CCC.h"
#include "Pixy2Line.h"
#include "Pixy2Video.h"

#include <Arduino.h>
#include <Pixy2.h>

namespace PixySig {
    constexpr uint16_t kBall       = 1;
    constexpr uint16_t kYellowGoal = 2;
    constexpr uint16_t kBlueGoal   = 3;
}

// Pixy2 native frame: 316 x 208 px (x: 0-315, y: 0-207)
namespace PixyFrame {
    constexpr uint16_t kWidth        = 316;
    constexpr uint16_t kHeight       = 208;
    constexpr uint16_t kCenterX      = kWidth / 2;
    constexpr uint16_t kCenterY      = kHeight / 2;
    constexpr uint16_t kShootThreshY = 190;
}

struct PixyBlock {
    bool found;
    double angle;   // 0 means straight ahead, negative is left, positive is right.
    uint16_t x;     // Raw frame X (0-315).
    uint16_t y;     // Raw frame Y (0-207).
    uint32_t area;  // width x height, largest block wins.
};

// Call once in setup(); halts on failure.
void pixyInit();

// Shared "not found" value used when no goal target is available yet.
PixyBlock pixyNoGoalBlock();

// Returns the largest block matching signature.
// found=false if nothing detected.
PixyBlock pixyGetBlock(uint16_t signature);

// Compares two candidate goal signatures and returns the one with the larger
// visible block. Returns 0 when neither goal is visible.
uint16_t pixyChooseGoalSignature(uint16_t firstGoalSignature, uint16_t secondGoalSignature);

// Locks onto one of two possible goals using the shared choose logic.
// Returns true once a goal signature is stored in lockedGoalSignature.
bool pixyLockGoalSignature(uint16_t &lockedGoalSignature,
                       uint16_t firstGoalSignature,
                       uint16_t secondGoalSignature,
                       unsigned long timeoutMs = 0,
                       unsigned long *lastSeenTime = nullptr,
                       const char *lockLabel = nullptr);

// Reads the currently locked goal, or returns an empty block when none
// has been selected yet.
PixyBlock pixyReadLockedGoal(uint16_t lockedGoalSignature);

// Shapes a detected goal angle into a drive correction with optional direction
// flip, clamp, and deadband. Returns 0 when the goal is missing or inside deadband.
float pixyGetGoalDriveAngle(const PixyBlock &goalBlock,
                            float clampDeg,
                            float directionSign = 1.0f,
                            float deadbandDeg = 0.0f);

bool pixyIsGoalCentered(const PixyBlock &goalBlock, uint16_t tolerancePx);
bool pixyIsGoalInsideLane(const PixyBlock &goalBlock, int leftX, int rightX);
bool pixyIsGoalYInRange(const PixyBlock &goalBlock, uint16_t minY, uint16_t maxY);
bool pixyIsGoalCloseEnough(const PixyBlock &goalBlock, uint32_t minArea);
double pixyGetHeadingTargetForGoal(const PixyBlock &goalBlock,
                                   double currentYaw,
                                   uint32_t minArea,
                                   float angleDeadbandDeg = 0.0f);

extern Pixy2 pixy;
