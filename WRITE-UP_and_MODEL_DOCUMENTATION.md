Write-up including : 
- The rubric points.
  - Brief description of how I addressed each point and references to the relevant code.
- Detailed description of the code used in each step (with line-number references and code snippets where appropriate) 
- Links to other supporting documents or external references.
- Explain how the code works and why I wrote it that way.

# Project Rubric Criteria

| Criteria Compilation| Meets Specifications |
|-----------|----------------|
|Code compiles correctly.|Code must compile without errors with cmake and make.|

```
./buildremi.sh 
-- Configuring done
-- Generating done
-- Build files have been written to: /home/workspace/CarND-Path-Planning-Project/build
[100%] Built target path_planning
```

| Criteria Valid Trajectories| Meets Specifications |
|-----------|----------------|
|Car able to drive at least 4.32 miles without incident| This implementation allows the car to drive more than one lap without incidents as defined in below criterias|
|car drives according to the speed limit| The implementation set the maximum speed to 49.5 mph for the car, ensuring the car does not exceed the speed limit of 50 mph. It also ensures to not drive slower than speed limit by always keep accelerating until 49.5 mph unless car becomes too close to any cars ahead in the same lane. This is done thanks to the following pieces of code spread in different files/functions :|

| Criteria Valid Trajectories| Meets Specifications |
|Max Acceleration and Jerk are not Exceeded|The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3|
|Car does not have collisions|The car must not come into contact with any of the other cars on the road|
|car stays in its lane, except for the time between changing lanes|The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road|
|car is able to change lanes|The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic|

| Criteria Reflection| Meets Specifications |
|-----------|----------------|
|There is a reflection on how to generate paths|The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation|
|||
|||
|||
