package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.List;

/**
 * Autonomous OpMode named "secondaryparking".
 * Logic:
 * 1. Power up the shooter flywheel (spin-up only - gate stays closed, nothing fires) and
 *    watch for an AprilTag for up to APRILTAG_WAIT_SECONDS.
 * 2. If a tag is seen in that window, fire the shooter, then run the parking movement.
 * 3. If no tag is seen (including if the webcam isn't mounted/configured, which just means
 *    initAprilTag() never finds any detections), skip firing entirely and just run the
 *    parking movement - 60cm backward, then 180cm to the left (reversed from the original
 *    forward/right parking path).
 */
@Autonomous(name = "secondaryparking", group = "StarterBot")
public class SecondaryParking extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.5;
    public static final double CM_PER_SECOND = 71.33; // 214cm / 3.0s

    // How long to watch for an AprilTag before giving up and just parking.
    public static final double APRILTAG_WAIT_SECONDS = 15.0;

    // How long to run the intake/feeder group once we decide to fire.
    public static final double SHOOT_DURATION_SECONDS = 3.0;

    // Calibration for straight line
    public static final double LEFT_P_SCALE = 0.959;
    public static final double RIGHT_P_SCALE = 1.00;

    // Ramping constants to fix jerk and drift
    public static final long RAMP_UP_TIME_MS = 500;
    public static final long RAMP_DOWN_TIME_MS = 500;

    // Compensation offsets
    public static final double START_COMPENSATION = 0.0;
    public static final double STOP_COMPENSATION = 0.08;

    // feederServo is a ranged (positional) servo: "bottom" is its closed/resting position, and
    // it moves up FEEDER_SERVO_ROTATE_DEGREES to its open ("firing") position.
    public static final double FEEDER_SERVO_FULL_RANGE_DEGREES = RobotHardware.FEEDER_SERVO_FULL_RANGE_DEGREES;
    public static final double FEEDER_SERVO_ROTATE_DEGREES = 90.0;
    public static final double FEEDER_SERVO_DEGREES_PER_SECOND = 300.0;
    public static final double FEEDER_SERVO_BOTTOM_POSITION = 0.0;
    public static final double FEEDER_SERVO_OPEN_POSITION =
            FEEDER_SERVO_BOTTOM_POSITION + (FEEDER_SERVO_ROTATE_DEGREES / FEEDER_SERVO_FULL_RANGE_DEGREES);
    public static final double FEEDER_SERVO_ROTATE_DURATION_SECONDS =
            FEEDER_SERVO_ROTATE_DEGREES / FEEDER_SERVO_DEGREES_PER_SECOND;

    // pollenFeederServo: rests at 120 degrees and should move down to 20 degrees - identical to
    // what happens while Square is held in MecanumStraferChassis - the instant an AprilTag is
    // detected, then release back to resting once we're done with the shoot/skip decision.
    public static final double POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES = RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES;
    public static final double POLLEN_FEEDER_SERVO_RESTING_DEGREES = 120.0;
    public static final double POLLEN_FEEDER_SERVO_PRESSED_DEGREES = 20.0;
    public static final double POLLEN_FEEDER_SERVO_RESTING_POSITION =
            POLLEN_FEEDER_SERVO_RESTING_DEGREES / POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES;
    public static final double POLLEN_FEEDER_SERVO_PRESSED_POSITION =
            POLLEN_FEEDER_SERVO_PRESSED_DEGREES / POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES;

    // Distance calibration scale factor - matches MecanumStraferChassis's AprilTag range calibration.
    public static final double DISTANCE_CALIBRATION_SCALE = (160.0 / 145.0) * 1.02;
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

        // Keep the feeder gate closed so nothing can fire while the shooter spins up and
        // we're watching for an AprilTag below.
        robot.setFeederServoPosition(FEEDER_SERVO_BOTTOM_POSITION);
        // pollenFeederServo starts resting (released), same as Square released in MecanumStraferChassis.
        robot.setPollenFeederServoPosition(POLLEN_FEEDER_SERVO_RESTING_POSITION);

        // Initialize Vision (try-catch since the webcam may not be mounted/configured on this
        // robot yet - if it isn't, aprilTag stays null and we simply never "see" a tag below,
        // which falls through to the no-shoot / parking-only behavior).
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "Initialized. Name: secondaryparking");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Power up the shooter flywheel immediately so it has time to reach speed, but do
            // NOT fire yet - the feeder gate stays closed and the intake group stays off.
            robot.setTargetShooterRPM(RobotHardware.DEFAULT_SHOOTER_RPM);

            telemetry.addData("Status", "Shooter spinning up - watching for AprilTag");
            telemetry.update();

            boolean tagSeen = false;
            int seenTagId = -1;
            ElapsedTime tagTimer = new ElapsedTime();
            tagTimer.reset();

            while (opModeIsActive() && !tagSeen && tagTimer.seconds() < APRILTAG_WAIT_SECONDS) {
                if (aprilTag != null) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    double closestDistanceCm = getClosestAprilTagDistanceCm(currentDetections);
                    if (!currentDetections.isEmpty()) {
                        tagSeen = true;
                        AprilTagDetection firstDetection = currentDetections.get(0);
                        if (firstDetection instanceof AprilTagSingleDetection) {
                            seenTagId = ((AprilTagSingleDetection) firstDetection).id;
                        }
                        // Make the detection obvious on the Driver Hub as soon as it happens.
                        telemetry.addData("AprilTag Status", "SEEN! (ID %d) - firing", seenTagId);
                        // Mirror MecanumStraferChassis's Square-held pollenFeederServo behavior
                        // now that a tag has actually been detected.
                        robot.setPollenFeederServoPosition(POLLEN_FEEDER_SERVO_PRESSED_POSITION);
                    } else {
                        telemetry.addData("AprilTag Status", "Watching for AprilTag...");
                    }
                    telemetry.addData("Camera State", visionPortal.getCameraState());
                    telemetry.addData("Raw Detections Count", currentDetections.size());
                    if (closestDistanceCm >= 0) {
                        telemetry.addData("Closest AprilTag Distance", "%.2f cm", closestDistanceCm);
                    } else {
                        telemetry.addData("Closest AprilTag Distance", "No pose data yet");
                    }
                } else {
                    telemetry.addData("AprilTag Status", "Camera/Processor not initialized (no webcam)");
                }

                telemetry.addData("Timer", "%.1f / %.0fs", tagTimer.seconds(), APRILTAG_WAIT_SECONDS);
                telemetry.addData("shooter Motor", "RPM: %.1f", robot.getMotorRPM());
                telemetry.update();

                idle();
            }

            if (tagSeen) {
                // Hold this message on the Driver Hub for a moment so it's clearly visible
                // before the firing telemetry below starts overwriting it.
                telemetry.addData("AprilTag Status", "SEEN! (ID %d) - Firing.", seenTagId);
                telemetry.update();
                sleep(500);
                fireShooter(seenTagId);
            } else {
                telemetry.addData("AprilTag Status", "NOT seen within %.0fs - skipping shot", APRILTAG_WAIT_SECONDS);
                telemetry.update();
            }

            // Whether or not we fired, the shooter isn't needed anymore before parking.
            robot.setTargetShooterRPM(0.0);

            // Release pollenFeederServo back to its resting position, mirroring releasing
            // Square in MecanumStraferChassis.
            robot.setPollenFeederServoPosition(POLLEN_FEEDER_SERVO_RESTING_POSITION);

            // Parking movement always runs, reversed from the original path: 60cm backward,
            // then 180cm to the left.
            telemetry.addData("Status", "Moving 60cm backward");
            telemetry.update();

            moveBackward(60.0);
            stopRobot();
            sleep(250);

            telemetry.addData("Status", "Moving 180cm left");
            telemetry.update();

            moveLeft(180.0);
            stopRobot();

            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(500);
        }

        // Clean up vision portal resource when OpMode is done
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /** Opens the feeder gate and runs the intake group for SHOOT_DURATION_SECONDS to fire. */
    private void fireShooter(int seenTagId) {
        if (robot.feederServo != null) {
            telemetry.addData("AprilTag Status", "SEEN! (ID %d)", seenTagId);
            telemetry.addData("Status", "Moving feederServo to open position (90 degrees up)");
            telemetry.update();
            robot.setFeederServoPosition(FEEDER_SERVO_OPEN_POSITION);
            sleep((long) (FEEDER_SERVO_ROTATE_DURATION_SECONDS * 1000));
        }

        ElapsedTime fireTimer = new ElapsedTime();
        fireTimer.reset();

        while (opModeIsActive() && fireTimer.seconds() < SHOOT_DURATION_SECONDS) {
            robot.setIntakePower(1.0);
            robot.setTargetShooterRPM(RobotHardware.DEFAULT_SHOOTER_RPM);

            telemetry.addData("AprilTag Status", "SEEN! (ID %d)", seenTagId);
            telemetry.addData("Status", "Firing");
            telemetry.addData("Timer", "%.1f / %.1fs", fireTimer.seconds(), SHOOT_DURATION_SECONDS);
            telemetry.update();

            idle();
        }

        robot.setIntakePower(0.0);
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

    private void setPowerWithSteer(double power, double steer) {
        double leftPower = (power - steer) * LEFT_P_SCALE;
        double rightPower = (power + steer) * RIGHT_P_SCALE;

        robot.frontLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        robot.frontRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
        robot.backLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        robot.backRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
    }

    private void stopRobot() {
        robot.stopDrive();
    }

    /**
     * Scans all current detections and returns the calibrated distance (cm) of the closest one,
     * or -1.0 if none of the detections have pose data yet.
     */
    private double getClosestAprilTagDistanceCm(List<AprilTagDetection> detections) {
        double closestDistanceCm = -1.0;
        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null) {
                double range = detection.ftcPose.range * DISTANCE_CALIBRATION_SCALE;
                if (closestDistanceCm < 0 || range < closestDistanceCm) {
                    closestDistanceCm = range;
                }
            }
        }
        return closestDistanceCm;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
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
