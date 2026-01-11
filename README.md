To launch the entire system (all nodes and the simulation), run the following command in your workspace:

```bash
ros2 launch assignment2py_rt assignment2_launch.py
```

Once the system is running, the UI Node will open a menu in the terminal with four options:

1. Set New Velocity: Allows the user to input linear and angular velocities. The robot will maintain this velocity indefinitely unless:
- The user sends a Stop command.

- The user sets a New Velocity.

- The Safety System detects an obstacle closer than the set threshold. In this case, the robot will automatically enter in "Emergency Phase", temporarily overriding user commands.

2. STOP Robot: Immediately stops the robot by sending a null velocity message (all components set to zero).

3. Change Safety Threshold (Service): Calls a ROS 2 service (/set_threshold) to update the minimum distance the robot must maintain from obstacles. The SafetyNode updates its behavior in real-time based on this new value.

4. Get Average Velocity (Service): Calls a service (/get_avg_speed) that calculates and displays the average linear and angular velocity of the most recent 5 user inputs (only from Option 1). Commands from Option 2 (Stop) are not included in this calculation.

While the robot is moving, the Safety Node constantly publishes feedback on the /robot_status topic. This custom message provides real-time information:

- Distance: The current distance to the nearest obstacle detected by the laser scanner.

- Direction: The sector where the closest obstacle is located (front, left, right, or back).

- Threshold: The active safety distance limit.

You can observe this data by running:
```bash
ros2 topic echo /robot_status
```

The safety system is based on a non-blocking State Machine. When the distance to an obstacle falls below the threshold, the robot automatically overrides user commands and enters the "Emergency Phase":

- Rotation State: The robot stops its linear movement and starts rotating. It will continue to spin until the closest obstacle is detected in the back sector.

- Escape State: Once the obstacle is safely behind the robot, it moves forward with a positive linear velocity for a set amount of time.

- Safe State: After moving away from the danger zone, the robot stops completely and resets the user's last command to zero, waiting for a new input from the menu.

Design Choice: This approach was chosen as the most straightforward and realistic solution to ensure safety. It avoids "teleporting" the robot back to a previous coordinate, relying instead on physical movement to reach a safe area, which better simulates real-world robotic behavior.
