# Implementation Plan - Update CoachDemoAutonomous

Update `CoachDemoAutonomous.java` to incorporate AprilTag detection and specific hardware configurations, then implement the requested autonomous logic.

## User Review Required

> [!IMPORTANT]
> The hardware names used ("Webcam 1", "feederMotor", "shooterMotor", etc.) must match the configuration on the actual robot. I will be using the names found in `SkongAutonomous.java` and `MecanumStraferChassis.java`.

## Proposed Changes

### Hardware & Vision Integration

#### [MODIFY] [CoachDemoAutonomous.java](file:///C:/Development/ftc/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/CoachDemoAutonomous.java)
- **Imports**: Add necessary imports for `VisionPortal`, `AprilTagProcessor`, `CRServo`, and other hardware classes.
- **Hardware Declarations**: Add `feeder`, `shooter`, `leftServo`, and `rightServo` motors/servos.
- **Vision Members**: Add `VisionPortal` and `AprilTagProcessor` fields.
- **Initialization**:
    - Update motor/servo initialization to match `MecanumStraferChassis.java`.
    - Implement `initAprilTag()` as seen in `SkongAutonomous.java`.
- **Logic**:
    - Start the `feeder`, `leftServo`, and `rightServo` immediately after `waitForStart()`.
    - Implement a loop that moves the robot forward.
    - Inside the loop, check for AprilTag detections.
    - Once a tag is detected:
        - Stop the drive motors.
        - Start the `shooter` motor at full power.
        - The `feeder` mechanism continues to run.
        - Terminate the drive loop and enter a "fire" state.

## Verification Plan

### Automated Tests
- Build the project to ensure no syntax errors or missing dependencies.

### Manual Verification
- Deploy to the robot.
- Verify that the feeder starts spinning immediately.
- Verify that the robot moves forward until it sees an AprilTag.
- Verify that once a tag is detected, the robot stops and the shooter spins up while the feeder stays running.
