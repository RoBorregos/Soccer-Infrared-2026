# Soccer-Infrared-2026
## Equipo
Luis Alfonzo Ramírez - *Mecanico*

Eduardo Mateo Murillo  - *Electrónico*

Aaron Flores - *Programador*

Andrés Rodríguez Cantú — *Programador*



## Documentation for Phototransistor and MUX library

To be able to play legally by the rules of RCJ soccer the robot may not leave the lines of the field or cross the penalty lines which prompts a 30 second lack of progress, precious time lost that makes it very hard to win.

Phototransistors are electronic devices that react to how much light goes into it at any point in time, it communicates this to a microcontroller by sending voltage signals going higher if there is more light, naturally we use this to our advantage as the white lines reflect more light than the green field.
To achieve this we designed a system based on arrays of phototransistors on lower PCBs which communicate each individual value via a multiplexer that controls which channel sends data using this we get extremely precise values for each phototransistor.
We use this data and individual channel tresholds to know when we hit line, to avoid having to calibrate every single time on different lightning and color surfaces we used a delta method which simplifies line detection we set maximum delta to filter out electric noise and only say we are on line when the delta (difference between current and last reading) crosses that treshold, treshold we have founnd to be very consistent in different lightning and color as even if the light is higher or the color is lighter the change between white and green will almost always be the same.
Phototransistor PCBs are some of the most complicated pieces of hardware of our robot equipped with leds to make lightning detection possible the resistors and voltage they get is extremely important as very low resistance makes leds be brighter but creates an important problem we found, the phototransistors take longer to filter out higher values when they detect line making it harder to ve precise and reliable.
We also tested different LED colors, White and Red, red proved to show a much lower noise rate on the values keeping them closer and extremely consistent while white was more erratic and jumped to higher deltass inconsistently.
White also had the advantage that white LEDs can use higher resistance which helps the phototransistors to drown out voltage spikes faster when detecting line.
We ended up using a mix of both due to component limitations but they are still reliable and consistent when the hardware is done correctly.

## Pixy libraries and usage 
To elevate our game we use a Pixy2 cam on each of our robots, the striker has it mounted to the front while the goalie has it on the back.
The purpose of this is to integrate goal detection for opponent and own goal respectively for each of our robots to do their jobs better.

To do all we do using the Pixy vision signatures were trained using the very intuitive software PixyMonV2 which easily lets you pick a color signature, name it, choose how sensitive it is, the size or number of blocks in frame and a lot of camera settings to make the scene brighter, darker or add more contrast all to make signature detection easier.
After picking and choosing all the settings and signatures you then save the parameters and can easily apply them to any Pixy quickly with the local file created with the parameters, this parameteres are stored internally inside the Pixy which is what allows our Teensy to Arduino/Teensy communication.

Usage

-Striker
The main usage of the Pixy on the striker is to detect the opponents goal signature given by:
SIG2 = YELLOW_GOAL and SIG3 BLUE_GOAL both these signatures are easily picked up on startup and locked on so the robot always knows what goal its shooting at.
Ball detection using SIG1 = BALL was a big idea we had but having two different versions of where the ball was using the IR ring and the Pixy was a bit conflictive and hard to deal with so we decided to mount the pixy in a way that allows constant almost full pitch goal detection to orient our robot to actually shoot at the goal instead of mindlessly chasing the ball.

-Goalie
As documented contrasting with our striker the goalie has a much conservative style of play staying back imitatint the ball angle with the IR ring while aligning itself with its own goal always, picking what its own goal will be on startup and protecting it at all cost.
Pixy makes this entire logic possible sending the position and size of the goal to keep our goalie correctly positioned to stop any ball coming to our goal.
With the goalie there was never any intention on using Pixy to see the ball even if there is a possibilty to do so the extra data flowing into it would slow down much more important tasks while giving us pretty much useless data on the ball behind us.

## Main striker and goalie workflows

Colibri
Like the name says our striker is the more jittery, fast and unpredictable robot of the 2, it moves using extremely precise IR calculations and PID orientation using a complex control system to keep it facing forward at all times making "el espacio como d luna llena" be able to carry the ball into the goal
Colibri runs on a loop of checking for line first then checking for PID orientation then checking for IR ball calculated angle and attacking it, finally the robot uses the header PID to angle itself toward the goal whenever its visible using the Pixy as explained in "Pixy libraries and usage" this makes for a very aggresive robot designed to keep the ball on the opponents side and angle itself toward the goal to create more shots on goal and opportunities. 
Its design has flaws like possible own goals in certain situations and moving too fast for the phototransistors to detect the line stopping us from being able to use higher speed from the motors which would allow for much more aggresive play.

Ajolote
Reflecting its name our goalie is smarter and has an exotic style of play, it stays behind without being too conservative or too aggresive.
Ajolote runs on a loop very similar, checking for line first, then checking IR and Pixy to position itself between the goal and the ball and if the ball gets too close it attacks it aggresively but ready to go back as soon as the goal is too far keeping a sort of safe zone to protect our goal from stray balls or from other robots attacking the ball. 


## Current software summary

Current striker software behavior
- On startup it captures phototransistor baseline values and locks onto the opponent goal using Pixy `SIG2` and `SIG3`.
- During play it checks line first, then keeps PID heading stable with the IMU, then uses the UART IR ball angle to chase the ball.
- When the ball is centered enough and the opponent goal is visible, it uses Pixy goal position to aim before pushing forward to shoot.
- If the ball angle data is lost, it stops translation and only keeps heading correction.

Current goalie software behavior
- On startup it captures phototransistor baseline values and locks onto its home goal using Pixy `SIG2` and `SIG3`.
- During play it checks line first, then keeps PID heading stable with the IMU, then uses Pixy to stay centered with its own goal.
- If the ball angle becomes threatening and the goalie is still inside its safe home-goal zone, it slides to intercept.
- If the goalie drifts too far from its goal or loses goal tracking, it retreats and re-centers before defending again.

## Startup behavior

Startup sequence
- The phototransistor system is initialized first.
- Margins for line detection are loaded from `constants.h`.
- A baseline capture is made using several samples so the robot can compare current readings against its starting field condition.
- Motors are held stopped for a short delay so sensor startup is more stable.
- The Pixy then tries to lock one of the two goal signatures.
- The IMU heading is read and stored as the target yaw used by heading PID during movement.

Why this matters
- If baseline capture is bad, line detection will be noisy or late.
- If goal lock fails at startup, Pixy-based positioning and aiming become weaker until the robot sees the goal again.
- If IMU startup is unstable, the robot will drift or fight itself while moving.

## Main thresholds to tune

Striker
- `Constants::Striker::kIRPossessionAngleToleranceDeg`: how centered the ball must be before the striker starts aiming the goal instead of only chasing.
- `Constants::Striker::kGoalCenterTolerancePx`: how centered the goal must be in the Pixy frame before the striker considers it lined up.
- `Constants::Striker::kGoalAimAreaThreshold`: how large the goal must look before the striker commits more directly to shooting.
- `Constants::Striker::kIRBallAngleClampDeg`: limits how sharply the striker can cut while chasing.
- `Constants::Striker::kGoalAngleClampDeg`: limits how aggressively Pixy can pull the striker left or right during aiming.
- `Constants::Striker::kChaseDrivePwmRatio`, `kGoalDrivePwmRatio`, `kAvoidDrivePwmRatio`: overall behavior balance between control, aiming, and line escape.

Goalie
- `Constants::Goalie::kGoalAngleClampDeg`: maximum Pixy correction while centering on the home goal.
- `Constants::Goalie::kGoalCenterDeadbandDeg`: how much goal error is tolerated before the goalie starts correcting.
- `Constants::Goalie::kGoalCorrectionWeight`: how strongly goal-centering affects intercept movement.
- `Constants::Goalie::kBallFollowWeight`: how strongly ball angle affects intercept movement.
- `Constants::Goalie::kIRThreatAngleToleranceDeg`: how wide the threat window is before the goalie decides to react.
- `Constants::kLeftGoalKeeperTresholdX`, `kRightGoalKeeperTresholdX`, `kMinGoalKeeperTresholdY`, `kMaxGoalKeeperTresholdY`: the safe zone used to decide whether the goalie is still protecting the goal correctly.

Phototransistors and line behavior
- `Constants::kPhotoMargins`: main line detection tuning values for each channel.
- `Constants::kAvoidDurationMs`: how long the robot keeps escaping once line is detected.
- `Constants::kBaselineSamples` and `Constants::kBaselineDelayMs`: baseline stability during startup calibration.

## What we want to improve next

Main improvement goals
- fewer line mistakes
- faster recovery after losing the ball
- more stable heading while moving
- better decision on when to shoot versus when to keep carrying the ball

Ideas to improve this
- Improve striker shot selection so it only commits hard to goal when the ball is centered and the goal is stable in Pixy.
- Add more recovery behavior after losing ball angle so the robot does not freeze too long or drift into bad positions.
- Make heading control more consistent while translating fast, especially when switching between chase and goal-aim modes.
- Reduce false line triggers and missed line detections by retuning channel margins and testing on different field lighting.
- Retune goalie safe-zone thresholds so it defends aggressively without drifting too far from goal.
- Add more repeated field tests for corner balls, side entries, rebounds, and traffic from other robots.

Suggested tuning workflow
- A good next step is to use a concrete field-testing order: change 5 important constants one by one and watch exactly what behavior improves or gets worse.


