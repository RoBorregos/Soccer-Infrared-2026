#ifndef ARDUINO_PIXYCAM_H
#define ARDUINO_PIXYCAM_H

#include <Arduino.h>
#include <Pixy2.h>

#define FRAME_W           316     
#define FRAME_H           208     
#define FRAME_CENTER_X    158     
#define FRAME_CENTER_Y    104     

#define SIG_YELLOW_GOAL   3       
#define SIG_BLUE_GOAL     2       
#define SIG_BALL          1       

// GOALPOST DETECTION THRESHOLDS
#define GOAL_MIN_WIDTH    10      
#define GOAL_MIN_HEIGHT   10      
#define GOAL_MIN_SCORE    0       

#define GOAL_DISTANCE_K   3000.0  // estimated_distance = K / blob_width

extern Pixy2 pixy;
#endif 