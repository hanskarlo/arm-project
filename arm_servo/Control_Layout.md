# Game Controller Controls Layout

<img src="https://pngimg.com/uploads/xbox/xbox_PNG17527.png" alt="MoveIt Logo" width="400"/>

## JointJog mode (Default)

|Controls           |                                |
|-------------------|--------------------------------|
| DPAD UP            | Speed up                                                    |
| DPAD DOWN          | Speed down                                                  |
| DPAD RIGHT         | Increment controlled joint                                  |
| DPAD LEFT          | Decrement controlled joint                                  |
| RIGHT/LEFT BUMPER  | Jog joint at constant speed                                 |
| RIGHT/LEFT TRIGGER | Jog joint at variable speed (proportional to trigger press) |
| START BUTTON       | Switch to CartesianJog                                      |


## CartesianJog mode

|Controls           |                                |
|-------------------|--------------------------------|
| DPAD UP           | Speed up                       |
| DPAD DOWN         | Speed down                     |
| RIGHT BUMPER      | Move in $+z$ direction         |
| LEFT BUMPER       | Move in $-z$ direction         |
| LEFT ANALOG STICK | Jog end effector in $xy$ plane |
| START BUTTON      | Switch to JointJog             |


## Modifying the controls
The game controller source code is found in `src/servo_game_controller.cpp`.

The `joy_cb()` method defines the buttons and their corresponding actions.