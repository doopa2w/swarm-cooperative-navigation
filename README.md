# swarm-cooperative-navigation
Cooperative Navigation for Swarm Robot in Indoor Environment

Issues are mostly spams of TODO for reminding myself what is lacking or what is a temporary fix that requires a proper implementation/ fix later on

Dependency
- C++
- CMAKE
- ARGos3
- Lua53
- Ubuntu 18.04

## TODO ##
1. ~~Range and Bearing Communication for both mobile and target robots~~
2. ~~Movement Pattern (Hard Turn, Soft Turn, No Turn)~~
3. ~~Robot's States -> Random Exploration, Goal Reaching Behaviour ...~~
4. ~~Odometry Information (Horizontal Bearing + Vertical Bearing + Distance Apart = Vector2 heading)~~
5. ~~Real to Byte and Vice versa Formatter~~
6. ~~Navigational Table~~
7. ~~Increment for SequenceNumber on outdated messages~~


## UNDECIDED ELEMENT ##

- Reaching goal via
    - Move to the sender's position first (The robot that sent the message/ table containing the
        designated goal info)
    - Once reached the sender's position (*The position where the message was sent), move towards
        the goal using the info 
    - Pros: The path from itself to sender is guranteed no obstacles (communication is LOS); Robots
        going to the same path (Similar to Ants behaviour)
    -Cons: Path used to go to goal might be congested -> more collision avoidance -> waste time;
        Requires additional implementation (a new state to move to sender based on sender's     relative position)

- OR
    - Add the vector to goal given by the sender and the vector between itself and the sender
        to get a new vector from itself to the goal
    - Update the goal info with the new vector
    - Move towards teh goal using the info (similar to previous)
    - Pros: Can still stick with the original implementation; May be not as congested compared to
        previous method
    - Cons: New path might be present with obstacles (undiscovered after all); Doesn't reflect what
        I imagined a swarm behaviour would do 








