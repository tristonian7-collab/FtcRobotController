package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.List;

/**
 * Autonomous OpMode named "primaryshooterauto".
 * Logic:
 * 1. Spin feeder for 10 seconds.
 * 2. Wheels are currently disabled for testing.
 */
@Autonomous(name = "primaryshooterauto", group = "StarterBot")
public class HorseAuto extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.5;
    public static final double CM_PER_SECOND = 71.33; // 214cm / 3.0s

    // Flywheel (shooter) power level
    public static final double SHOOTER_POWER = 0.56;

    public static final double TURN_SPEED = 0.33;
    public static final double TURN_SECONDS = 1.0;

    // Calibration for straight line
    public static final double LEFT_P_SCALE = 0.959;
    public static final double RIGHT_P_SCALE = 1.00;

    // Ramping constants to fix jerk and drift
    public static final long RAMP_UP_TIME_MS = 500;
    public static final long RAMP_DOWN_TIME_MS = 500;

    // Compensation offsets
    public static final double START_COMPENSATION = 0.0;
    public static final double STOP_COMPENSATION = 0.08;

    // AprilTag Tracking Constants
    public static final double DESIRED_DISTANCE = 12.0; // target distance in inches away from tag
    public static final double SPEED_GAIN = 0.03;       // Proportional gain for speed
    public static final double TURN_GAIN = 0.02;        // Proportional gain for steering

    // feederServo is a ranged (positional) servo, so "bottom" is an absolute position
    // (FEEDER_SERVO_BOTTOM_POSITION) rather than 0 power. After FEEDER_SERVO_BOTTOM_WAIT_MS
    // elapses, it moves to FEEDER_SERVO_OPEN_POSITION (90 degrees "up" from bottom).
    // FEEDER_SERVO_DEGREES_PER_SECOND estimates how long a position change takes to physically
    // complete so we can wait for it before continuing.
    // TODO: Calibrate FEEDER_SERVO_DEGREES_PER_SECOND to the actual servo's spec (deg/sec at
    // full speed); keep in sync with the same constant in MecanumStraferChassis.
    public static final long FEEDER_SERVO_BOTTOM_WAIT_MS = 4000;
    public static final double FEEDER_SERVO_FULL_RANGE_DEGREES = RobotHardware.FEEDER_SERVO_FULL_RANGE_DEGREES;
    public static final double FEEDER_SERVO_ROTATE_DEGREES = 90.0;
    public static final double FEEDER_SERVO_DEGREES_PER_SECOND = 300.0;
    public static final double FEEDER_SERVO_BOTTOM_POSITION = 0.0;
    public static final double FEEDER_SERVO_OPEN_POSITION =
            FEEDER_SERVO_BOTTOM_POSITION + (FEEDER_SERVO_ROTATE_DEGREES / FEEDER_SERVO_FULL_RANGE_DEGREES);
    public static final double FEEDER_SERVO_ROTATE_DURATION_SECONDS =
            FEEDER_SERVO_ROTATE_DEGREES / FEEDER_SERVO_DEGREES_PER_SECOND;

    // Gate (feederServo) end-of-auto motion: after the final movement, the gate moves up
    // GATE_ROTATE_DEGREES degrees from its open position, then immediately moves back down
    // the same amount.
    public static final double GATE_ROTATE_DEGREES = 25.0;
    public static final double GATE_UP_POSITION =
            FEEDER_SERVO_OPEN_POSITION + (GATE_ROTATE_DEGREES / FEEDER_SERVO_FULL_RANGE_DEGREES);
    public static final double GATE_ROTATE_DURATION_SECONDS =
            GATE_ROTATE_DEGREES / FEEDER_SERVO_DEGREES_PER_SECOND;
    // ----------------------------

    // Centralized hardware map shared across all OpModes
    private final RobotHardware robot = new RobotHardware();

    private final ElapsedTime runtime = new ElapsedTime();

    // --- Vision Members ---
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;

    @Override
    public void runOpMode() {

        // Initialize all hardware devices using the shared hardware map
        robot.init(hardwareMap);

        // Wait FEEDER_SERVO_BOTTOM_WAIT_MS before commanding feederServo at all, so it does not
        // immediately snap to its "bottom" (closed) position the instant the OpMode is INIT'd.
        sleep(FEEDER_SERVO_BOTTOM_WAIT_MS);

        // feederServo starts held at its "bottom" (closed) resting position; it will move to
        // its open position (90 degrees up) later, once FEEDER_SERVO_BOTTOM_WAIT_MS (measured
        // from waitForStart()) has elapsed in the shooting sequence below.
        robot.setFeederServoPosition(FEEDER_SERVO_BOTTOM_POSITION);

        // Initialize Vision (try-catch since camera may not be mounted/configured yet)
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "Initialized. Name: primaryshooterauto");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Flywheel (shooter) spins continuously at SHOOTER_POWER for the entire OpMode.
            // Start it immediately at the start of the OpMode so it has the full
            // FEEDER_SERVO_BOTTOM_WAIT_MS window to reach speed while the robot moves backward
            // and the gate (feederServo) stays closed.
            robot.setShooterPower(SHOOTER_POWER);

            // Move backward 5cm immediately as well, in parallel with the shooter spin-up and
            // gate wait below. feederServo stays held at its bottom (closed) position throughout.
            // (Reduced from the original 20cm by 15cm.)
            telemetry.addData("Status", "Spinning up shooter - moving 5cm backward");
            telemetry.addData("feeder Servo", "Holding at bottom (closed) position");
            telemetry.update();

            moveBackward(5.0);
            stopRobot();

            // feederServo stays held at its bottom (closed) position until FEEDER_SERVO_BOTTOM_WAIT_MS
            // has elapsed since the start of the OpMode (covers the backward move above).
            long elapsedSinceStartMs = (long) runtime.milliseconds();
            long remainingBottomWaitMs = FEEDER_SERVO_BOTTOM_WAIT_MS - elapsedSinceStartMs;
            telemetry.addData("Status", "Waiting for feeder bottom hold to reach 4.0s total");
            telemetry.update();
            if (remainingBottomWaitMs > 0) {
                sleep(remainingBottomWaitMs);
            }

            // After the bottom-position wait, move feederServo to its open position (90 degrees up).
            if (robot.feederServo != null) {
                telemetry.addData("Status", "Moving feederServo to open position (90 degrees up)");
                telemetry.update();
                robot.setFeederServoPosition(FEEDER_SERVO_OPEN_POSITION);
                sleep((long) (FEEDER_SERVO_ROTATE_DURATION_SECONDS * 1000));
            }

            // Spin the intake group (leftServo, rightServo, feederMotor) continuously for the
            // entire OpMode. These must always run together. feederServo is a positional gate
            // and simply holds at its open position (set above) for the rest of the OpMode.
            if (robot.feederMotor != null) {
                robot.feederMotor.setPower(1.0);
            }

            // Run for 10 seconds or until STOP is pressed
            ElapsedTime timer = new ElapsedTime();
            timer.reset();

            while (opModeIsActive() && timer.seconds() < 10) {
                // Intake group: leftServo, rightServo, feederMotor always together
                robot.setIntakePower(1.0);
                // Flywheel remains spinning continuously at SHOOTER_POWER
                robot.setShooterPower(SHOOTER_POWER);

                // feederServo gate was already moved clockwise to its open position after the
                // 4-second wait above and simply stays there for the rest of the shoot loop.

                // Show what the software thinks is happening
                telemetry.addData("Status", "feeder Running");
                telemetry.addData("Timer", "%.1f / 10.0s", timer.seconds());
                telemetry.addData("Servo L", "Power: %.2f", robot.leftServo.getPower());
                telemetry.addData("Servo R", "Power: %.2f", robot.rightServo.getPower());
                if (robot.feederServo != null) {
                    telemetry.addData("feeder Servo", "Position: %.2f", robot.feederServo.getPosition());
                } else {
                    telemetry.addData("feeder Servo", "NOT FOUND");
                }
                if (robot.feederMotor != null) {
                    telemetry.addData("feeder Motor", "Power: %.2f", robot.feederMotor.getPower());
                } else {
                    telemetry.addData("feeder Motor", "NOT FOUND");
                }
                telemetry.addData("shooter Motor", "Power: %.2f", robot.shooter.getPower());

                // AprilTag detection and distance reporting
                if (aprilTag != null) {
                    // Diagnostic: Check camera and processor state
                    telemetry.addData("Camera State", visionPortal.getCameraState());

                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    telemetry.addData("Raw Detections Count", currentDetections.size());
                    boolean tagFound = false;
                    for (AprilTagDetection detection : currentDetections) {
                        // In SDK v12.0+, any valid raw frame detection or cluster detection counts.
                        // We check if it has been fully localized/tracked by checking if code details are active.
                        tagFound = true;

                        if (detection instanceof AprilTagSingleDetection) {
                            AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                            String name = (singleDet.metadata != null) ? singleDet.metadata.name : "Unmapped Tag ID";
                            telemetry.addData("AprilTag Found", "ID %d (%s)", singleDet.id, name);
                        } else {
                            telemetry.addData("AprilTag Found", "Tag Cluster/Raw Frame");
                        }

                        if (detection.ftcPose != null) {
                            telemetry.addData("Distance (Range)", "%.2f inches", detection.ftcPose.range);
                            telemetry.addData("Bearing", "%.2f degrees", detection.ftcPose.bearing);
                        } else {
                            telemetry.addData("Distance (Range)", "Tracking pose... (Hold Still)");
                        }
                        break;
                    }

                    if (tagFound) {
                        // Tag found! Immediately halt all robot wheel movement.
                        stopRobot();
                        telemetry.addData("AprilTag Status", "Tag Detected! Stopped.");

                        // Flywheel is already spinning continuously at SHOOTER_POWER
                        telemetry.addData("Shooter Status", "Already spinning at %.2f power", SHOOTER_POWER);
                        telemetry.update();

                        // Break out of the loop completely once a tag is found to prevent it from starting again
                        break;
                    } else {
                        // Driving is disabled during the shoot loop - robot stays put and just shoots.
                        telemetry.addData("AprilTag Status", "No tags visible - Drive disabled (stationary)");
                        stopRobot();
                    }
                } else {
                    telemetry.addData("AprilTag Status", "Camera/Processor not initialized (not mounted or configured yet)");
                }

                telemetry.update();

                idle();
            }

            // Stop the drive train and intake group (leftServo, rightServo, feederMotor).
            // feederServo is a positional gate and simply remains at its current position.
            stopRobot();
            robot.setIntakePower(0.0);

            // Shooting is done - power down the flywheel before parking.
            robot.setShooterPower(0.0);

            telemetry.addData("Status", "Stopped - shooting complete");
            telemetry.update();
            sleep(500);

            // After firing and moving backward, move 130cm to the right, then 230cm backward.
            telemetry.addData("Status", "Moving 130cm right");
            telemetry.update();

            moveRight(130.0);
            stopRobot();
            sleep(250);

            telemetry.addData("Status", "Moving 200cm backward");
            telemetry.update();

            moveBackward(200.0);
            stopRobot();
            sleep(250);

            telemetry.addData("Status", "Moving 115cm right");
            telemetry.update();

            moveRight(115.0);
            stopRobot();

            telemetry.addData("Status", "Final move complete");
            telemetry.update();
            sleep(500);

            // After the last movement, move the gate (feederServo) 25 degrees up from its open
            // position, then immediately move it back down the same amount.
            if (robot.feederServo != null) {
                telemetry.addData("Status", "Gate: moving 25 degrees up");
                telemetry.update();
                robot.setFeederServoPosition(GATE_UP_POSITION);
                sleep((long) (GATE_ROTATE_DURATION_SECONDS * 1000));

                telemetry.addData("Status", "Gate: moving back down");
                telemetry.update();
                robot.setFeederServoPosition(FEEDER_SERVO_OPEN_POSITION);
                sleep((long) (GATE_ROTATE_DURATION_SECONDS * 1000));
            }
        }

        // Clean up vision portal resource when OpMode is done
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void moveForward(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm", distanceCm);
        telemetry.update();

        // 1. Ramp up
        startMoving(DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopMoving(DRIVE_SPEED);
    }

    private void moveBackward(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm backward", distanceCm);
        telemetry.update();

        // 1. Ramp up
        startMoving(-DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopMoving(-DRIVE_SPEED);
    }

    private void moveRight(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm right", distanceCm);
        telemetry.update();

        // 1. Ramp up (positive strafe = right, per RobotHardware#driveMecanum convention)
        startStrafing(DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopStrafing(DRIVE_SPEED);
    }

    private void moveLeft(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm left", distanceCm);
        telemetry.update();

        // 1. Ramp up (negative strafe = left, per RobotHardware#driveMecanum convention)
        startStrafing(-DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopStrafing(-DRIVE_SPEED);
    }

    private void startStrafing(double targetStrafePower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_UP_TIME_MS) {
            double ratio = rampTimer.milliseconds() / RAMP_UP_TIME_MS;
            double currentPower = targetStrafePower * ratio;
            robot.driveMecanum(0.0, currentPower, 0.0);
        }
        robot.driveMecanum(0.0, targetStrafePower, 0.0);
    }

    private void stopStrafing(double startingPower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_DOWN_TIME_MS) {
            double ratio = 1.0 - (rampTimer.milliseconds() / RAMP_DOWN_TIME_MS);
            double currentPower = startingPower * ratio;
            robot.driveMecanum(0.0, currentPower, 0.0);
        }
        stopRobot();
    }

    private void startMoving(double targetPower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_UP_TIME_MS) {
            double ratio = rampTimer.milliseconds() / RAMP_UP_TIME_MS;
            double currentPower = targetPower * ratio;
            setPowerWithSteer(currentPower, START_COMPENSATION * ratio);
        }
        setPowerWithSteer(targetPower, 0);
    }

    private void stopMoving(double startingPower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_DOWN_TIME_MS) {
            double ratio = 1.0 - (rampTimer.milliseconds() / RAMP_DOWN_TIME_MS);
            double currentPower = startingPower * ratio;
            setPowerWithSteer(currentPower, STOP_COMPENSATION * ratio);
        }
        stopRobot();
    }

    private void setPowerWithSteer(double power, double steer) {
        double leftPower = (power - steer) * LEFT_P_SCALE;
        double rightPower = (power + steer) * RIGHT_P_SCALE;

        robot.frontLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        robot.frontRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
        robot.backLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        robot.backRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
    }

    private void turnLeft(double degrees) {
        // Use perfected data: TURN_SPEED (0.33) and TURN_SECONDS (1.0 for 90 deg)
        robot.frontLeftDrive.setPower(-TURN_SPEED);
        robot.frontRightDrive.setPower(TURN_SPEED);
        robot.backLeftDrive.setPower(-TURN_SPEED);
        robot.backRightDrive.setPower(TURN_SPEED);

        sleep((long)((degrees / 90.0) * TURN_SECONDS * 1000));
        stopRobot();
        sleep(200);
    }

    private void turnRight(double degrees) {
        // Use perfected data: TURN_SPEED (0.33) and TURN_SECONDS (1.0 for 90 deg)
        robot.frontLeftDrive.setPower(TURN_SPEED);
        robot.frontRightDrive.setPower(-TURN_SPEED);
        robot.backLeftDrive.setPower(TURN_SPEED);
        robot.backRightDrive.setPower(-TURN_SPEED);

        sleep((long)((degrees / 90.0) * TURN_SECONDS * 1000));
        stopRobot();
        sleep(200);
    }

    private void stopRobot() {
        robot.stopDrive();
    }

    private void initAprilTag() {
        // Create a custom processor that allows ALL tags, even if they aren't in the default library
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        // Force decimation to 1.0 for maximum shape sensitivity
        aprilTag.setDecimation(1.0f);

        // Create the vision portal manually to ensure the custom processor binds completely
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
