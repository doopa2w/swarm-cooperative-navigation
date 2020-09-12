# swarm-cooperative-navigation
Cooperative Navigation for Swarm Robot in Indoor Environment

Dependency
- C++
- CMAKE
- ARGos3
- Lua53
- Ubuntu 18.04

## TODO ##
1. ~~Range and Bearing Communication for both mobile and target robots~~
2. Movement Pattern (Hard Turn, Soft Turn, No Turn)
3. Robot's States -> Random Exploration, Goal Reaching Behaviour ...
4. Odometry Information (Horizontal Bearing + Vertical Bearing + Distance Apart = Vector2 heading)
5. UTF-8 to Byte and Vice versa Formatter
6. Navigational Table

##  IDEA ##

### Message Payload Format/ Structure ###
* 4 bytes per char -> Have not decide on the size of message
* \0 terminating char for every collumn
* Message Structure: goalID \0 horizontal bearing \0 vertical bearing \0 relative distance \0
    * goalID             = range(1,4) -> 4 bytes
    * horizontal bearing =  idk       -> 24 bbytes
    * vertical bearing   =  idk       -> 24 bytes (4 d.p.)
    * relative distance  =  in cm     -> 28 bytes (up to 2000.00 cm) since we only doing the max 20mx20m arena
* ~ 77 bytes per goal/ row (Up to 4x77 = 308 bytes (3MB)) -> probably fine for simulation

### - EvaluateMessage(CByteArray MessagePayload, int GoalID) ###
Called right after getting readings from range and bearing.

* Should call formatter to translateCByteArray to UTF-8 string then to matrix
    * since its only used once, called the formatter inside this function
* 

### - EvaluateNavigationalTable() ###
* Navigational table format should now be a fixed size of array (fixed 4 rows for now)

### Things to look at the API
* CVector2(1.0f, cAccumulator.Angle()) from footbot_diffusion.cpp -> calculateVectorToLight
* DiffusionVector function (DiffusionParams); WithinMinBoundIncludedMaxBoundIncluded(diffVector)

# BIGGEST HEADACHE #

Should i set the vector<vector<Real>> with fixed size (configured inside XML) or not?

if fixed, no need to sort anymore; vector<vector<Real> > table (3); {{0, 0, 0, 0}, {1, 0, 0, 0}, {2, 0, 0, 0}};

![one_row_length](/home/dev/swarm-cooperative-navigation/one_row_length.png)

* Identifier for Which Type of Robot sent the message: 1 for Target/ 2 for Mobile (1 Byte long)
* A terminating char '\0': (4 Bytes long)
* Sequence Number: Age of info (4 bytes long)
* A terminating char '\0'': (4 bytes long)
* Range: Distance in Real (12 bytes long/ long  double)
* A terminating char '\0' (4 bytes long)
* Angle: Horizontal Bearing (CRadians) (12 bytes long)
* A terminating char '\0' for going next row (4 bytes long)

TOTAL FOR A ROW OF NAVIGATIONAL INFO: 1 + 4 + 4 + 4 + 12 +4 + 12 = 41 + A separator for next row (4 bytes) = 45 bytes long 



