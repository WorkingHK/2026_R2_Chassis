# PROJECT GOAL

    Build a mecanum-wheel mobile robot with:

    ESP32 as real-time motor & safety controller

    Damiao 3519 BLDC motors (CAN, speed mode)

    PS5 controller for manual override (always available)

    ROS 2 Humble (Orange Pi 5 Max) for autonomy (Nav2)

    micro-ROS bridge between ESP32 and ROS
    
    Wheel odometry + TF for localization

# Reference path
-> tested arduino code for driving 3519 motor [text](Reference/Esp32_mecanum_35199.ino)
https://github.com/rodneybakiskan/ps5-esp32/tree/main <-PS5 library
motor manaul:[text](<Reference/DM-S3519-1EC减速电机（含DM3520-1EC驱动器）使用说明书V1.1 (1).pdf>)



# SYSTEM RESPONSIBILITY SPLIT (Non-Negotiable)
Component	Responsibility
ESP32	Real-time control, safety, arbitration, CAN, odometry
Orange Pi 5 Max	Planning, SLAM, Nav2, UI
PS5	Manual emergency & teleop
ROS 2	High-level autonomy only

❌ ROS never directly controls motors
❌ Linux never handles safety

# DEVELOPMENT PHASES (STRICT ORDER)

Each phase must be working and tested before proceeding.

🔹 PHASE 0 — Hardware & Protocol Validation

Objective: Prove the hardware and CAN protocol are correct.

Tasks

Verify ESP32 ↔ CAN transceiver wiring

Confirm 1 Mbps CAN timing

Validate Damiao speed mode:

Enable command

Speed command (float, rad/s)

Verify motor direction consistency

Deliverables

Single-motor CAN test sketch

4-motor speed test sketch

Direction table (FL, FR, RL, RR)

Exit Criteria

✅ All motors spin reliably
✅ No CAN TX failures
✅ Speed unit confirmed

🔹 PHASE 1 — Mecanum Drive (ESP32 Only)

Objective: Make the robot drive correctly without ROS.

Tasks

Implement mecanum inverse kinematics

Hardcode test motions:

Forward

Strafe

Rotate

Enforce speed limits

Deliverables

mecanum(vx, vy, wz) function

Motor mapping document

Exit Criteria

✅ Robot moves correctly in all directions
✅ No unexpected rotations or drift

🔹 PHASE 2 — PS5 Manual Control (Safety First)

Objective: Add manual control that always overrides autonomy.

Tasks

Integrate PS5 Arduino library

Map:

Left stick → vx / vy

Right stick X → wz

Implement dead zones

Implement manual priority lock

Deliverables

PS5 control module

Manual override logic

Exit Criteria

✅ PS5 always overrides other commands
✅ Robot stops when controller disconnects
✅ No jitter or runaway

🔹 PHASE 3 — micro-ROS Communication Layer

Objective: Allow ROS to send velocity commands safely.

Tasks

Bring up micro-ROS agent on Orange Pi

ESP32 subscribes to /cmd_vel

Add timeout watchdog

Block ROS commands when PS5 connected

Deliverables

micro-ROS node on ESP32

/cmd_vel subscription

Arbitration FSM

Exit Criteria

✅ ROS teleop works
✅ PS5 overrides ROS instantly
✅ Robot stops on ROS timeout

🔹 PHASE 4 — Wheel Odometry (ESP32)

Objective: Publish correct odometry and TF for Nav2.

Tasks

Compute body velocity from wheel speeds

Integrate pose (x, y, yaw)

Publish:

/odom (nav_msgs/Odometry)

/tf (odom → base_link)

Ensure consistent frame naming

Deliverables

Odometry publisher

TF broadcaster

RViz visualization

Exit Criteria

✅ RViz shows correct movement
✅ No TF errors
✅ Odometry stable at constant speed

🔹 PHASE 5 — IMU Fusion (Recommended)

Objective: Reduce drift for navigation.

Tasks

Add IMU (e.g., BNO085 / ICM-20948)

Publish /imu

Configure robot_localization EKF

Fuse wheel odom + IMU yaw

Deliverables

IMU micro-ROS publisher

EKF config YAML

Exit Criteria

✅ Stable yaw
✅ Reduced drift
✅ Nav2 accepts odom

🔹 PHASE 6 — Nav2 Integration (Orange Pi)

Objective: Autonomous navigation.

Tasks

Configure robot footprint (mecanum)

Tune Nav2 controllers

Verify velocity limits

Test:

Go-to-pose

Recovery behaviors

Deliverables

Nav2 bringup package

Parameter files

Launch files

Exit Criteria

✅ Autonomous navigation works
✅ Smooth mecanum motion
✅ Manual override still works

🔹 PHASE 7 — Reliability & Safety Hardening

Objective: Make it real-world safe.

Tasks

Add hardware E-STOP

Add CAN fault detection

Detect PS5 reconnects

Add motor temperature / fault monitoring

Deliverables

Fault handler

Safety checklist

Exit Criteria

✅ Robot fails safely
✅ No uncontrolled motion possible

🧪 TESTING STRATEGY (MANDATORY)

For every phase:

Static test (wheels off ground)

Low-speed test

Full-speed test

Power-cycle test

Communication loss test