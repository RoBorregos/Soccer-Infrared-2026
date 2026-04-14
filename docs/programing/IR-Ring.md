# IR-Ring & sensor control.

Ball tracking is managed by a ring of 15 infrared (IR) recievers arranged in a pseudocircular confuguration; connected to a dedicated ATMEGA328P microcontroller. The system allows to detect the angle of the ball with respect to the center of the field, as well as a rough model to estimate the estate of where roughly the ball is on the field. The system, although inexact, it is able to provide a fast and most importantly, a reliable estimation of the ball position to later be used in PID.

The angle is mapped in the range of -180° to 180°, where 0° represents the front of the robot, negative values indicate the ball is to the right, and positive values to the left. This angular data is used to orient the robot towards the ball with high precision.

