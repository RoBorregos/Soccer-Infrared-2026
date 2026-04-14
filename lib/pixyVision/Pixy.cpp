#include "pixyLib.h"
#include <math.h>

Pixy2 pixy;

void pixyInit() {
    const int result = pixy.init();
    if (result == 0) {
        Serial.println(F("[SUCCESS] Pixy2 connected!"));
    } else {
        Serial.println(F("[FAIL] Pixy2 not found. Check wiring or power."));
        while (true) {
        }
    }
}

PixyBlock pixyNoGoalBlock() {
    return {false, 0.0, 0, 0, 0};
}

PixyBlock pixyGetBlock(uint16_t signature) {
    PixyBlock block = pixyNoGoalBlock();

    // Signature N maps to bit N-1 in the Pixy sigmap.
    const uint8_t sigmap = 1 << (signature - 1);

    const int8_t count = pixy.ccc.getBlocks(true, sigmap);
    if (count <= 0) {
        return block;
    }

    // Keep only the largest blob for the requested signature.
    int bestIndex = -1;
    uint32_t bestArea = 0;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        if (pixy.ccc.blocks[i].m_signature != signature) {
            continue;
        }

        const uint32_t area = static_cast<uint32_t>(pixy.ccc.blocks[i].m_width) *
                              static_cast<uint32_t>(pixy.ccc.blocks[i].m_height);
        if (area > bestArea) {
            bestArea = area;
            bestIndex = i;
        }
    }

    if (bestIndex < 0) {
        return block;
    }

    const uint16_t x = pixy.ccc.blocks[bestIndex].m_x;
    const uint16_t y = pixy.ccc.blocks[bestIndex].m_y;

    block.found = true;
    block.x = x;
    block.y = y;
    block.area = bestArea;

    const double dx = static_cast<double>(x) - static_cast<double>(PixyFrame::kCenterX);
    const double forwardBias = max(20.0, static_cast<double>(PixyFrame::kHeight - y));

    // Convert the blob offset into a simple steering angle around the camera center.
    block.angle = -atan2(dx, forwardBias) * 180.0 / PI;

    return block;
}

uint16_t pixyChooseGoalSignature(uint16_t firstGoalSignature, uint16_t secondGoalSignature) {
    const PixyBlock firstBlock = pixyGetBlock(firstGoalSignature);
    const PixyBlock secondBlock = pixyGetBlock(secondGoalSignature);

    if (firstBlock.found && secondBlock.found) {
        return (firstBlock.area >= secondBlock.area) ? firstGoalSignature : secondGoalSignature;
    }

    if (firstBlock.found) {
        return firstGoalSignature;
    }

    if (secondBlock.found) {
        return secondGoalSignature;
    }

    return 0;
}

bool pixyLockGoalSignature(uint16_t &lockedGoalSignature,
                       uint16_t firstGoalSignature,
                       uint16_t secondGoalSignature,
                       unsigned long timeoutMs,
                       unsigned long *lastSeenTime,
                       const char *lockLabel) {
    if (lockedGoalSignature != 0) {
        return true;
    }

    const unsigned long startTime = millis();

    do {
        lockedGoalSignature = pixyChooseGoalSignature(firstGoalSignature, secondGoalSignature);
        if (lockedGoalSignature != 0) {
            if (lastSeenTime != nullptr) {
                *lastSeenTime = millis();
            }

            if (lockLabel != nullptr) {
                Serial.print(F("Locked "));
                Serial.print(lockLabel);
                Serial.print(F(" signature: "));
                Serial.println(lockedGoalSignature);
            }

            return true;
        }

        delay(25);
    } while (timeoutMs > 0 && millis() - startTime < timeoutMs);

    return false;
}

PixyBlock pixyReadLockedGoal(uint16_t lockedGoalSignature) {
    if (lockedGoalSignature == 0) {
        return pixyNoGoalBlock();
    }

    return pixyGetBlock(lockedGoalSignature);
}

float pixyGetGoalDriveAngle(const PixyBlock &goalBlock,
                            float clampDeg,
                            float directionSign,
                            float deadbandDeg) {
    if (!goalBlock.found) {
        return 0.0f;
    }

    float angle = static_cast<float>(goalBlock.angle) * directionSign;
    angle = constrain(angle, -clampDeg, clampDeg);

    if (fabs(angle) <= deadbandDeg) {
        return 0.0f;
    }

    return angle;
}
