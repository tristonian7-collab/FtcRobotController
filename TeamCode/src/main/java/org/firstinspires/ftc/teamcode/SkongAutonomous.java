package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * Autonomous OpMode named "skongautonomous".
 * Logic:
 * 1. Spin feeder for 10 seconds.
 * 2. Wheels are currently disabled for testing.
 */
@Autonomous(name = "skongautonomous", group = "StarterBot")
public class SkongAutonomous extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.5;
    public static final double CM_PER_SECOND = 71.33; // 214cm / 3.0s

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
    // ----------------------------

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor feeder         = null;
    private CRServo leftServo = null;
    private CRServo rightServo = null;

    private final ElapsedTime runtime = new ElapsedTime();

    // --- Vision Members ---
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        try {
            feeder = hardwareMap.get(DcMotor.class, "feeder");
            feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Warning", "feeder motor 'feeder' not found");
        }

        // Set directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior to BRAKE
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Vision (try-catch since camera may not be mounted/configured yet)
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "Initialized. Name: skongautonomous");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Run for 10 seconds or until STOP is pressed
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            
            while (opModeIsActive() && timer.seconds() < 10) {
                // Set power to everything
                leftServo.setPower(1.0);
                rightServo.setPower(1.0);
                
                if (feeder != null) {
                    feeder.setPower(1.0);
                }

                // Show what the software thinks is happening
                telemetry.addData("Status", "feeder Running");
                telemetry.addData("Timer", "%.1f / 10.0s", timer.seconds());
                telemetry.addData("Servo L", "Power: %.2f", leftServo.getPower());
                telemetry.addData("Servo R", "Power: %.2f", rightServo.getPower());
                if (feeder != null) {
                    telemetry.addData("feeder Motor", "Power: %.2f", feeder.getPower());
                } else {
                    telemetry.addData("feeder Motor", "NOT FOUND");
                }

                // AprilTag detection and distance reporting
                if (aprilTag != null) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    boolean tagFound = false;
                    AprilTagDetection targetTag = null;
                    
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            telemetry.addData("AprilTag Found", "ID %d (%s)", detection.id, detection.metadata.name);
                            telemetry.addData("Distance (Range)", "%.2f inches", detection.ftcPose.range);
                            telemetry.addData("Bearing", "%.2f degrees", detection.ftcPose.bearing);
                            tagFound = true;
                            targetTag = detection;
                            break; // Target the first detected tag
                        }
                    }
                    
                    if (tagFound) {
                        telemetry.addData("AprilTag Status", "Tag Visible - Approaching");
                        
                        // Calculate error fields
                        double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;
                        double headingError = targetTag.ftcPose.bearing;
                        
                        // Compute power values using proportional gains
                        double drive = Range.clip(rangeError * SPEED_GAIN, -0.4, 0.4);
                        double turn = Range.clip(headingError * TURN_GAIN, -0.2, 0.2);
                        
                        // Small tolerances to prevent oscillating once under the tag
                        if (Math.abs(rangeError) < 1.0 && Math.abs(headingError) < 2.0) {
                            stopRobot();
                            telemetry.addData("Motion Status", "Arrived under AprilTag!");
                        } else {
                            setPowerWithSteer(drive, turn);
                        }
                    } else {
                        telemetry.addData("AprilTag Status", "No tags visible");
                        stopRobot();
                    }
                } else {
                    telemetry.addData("AprilTag Status", "Camera/Processor not initialized (not mounted or configured yet)");
                }

                telemetry.update();
                
                idle();
            }

            // Stop everything
            stopRobot();
            leftServo.setPower(0.0);
            rightServo.setPower(0.0);
            if (feeder != null) {
                feeder.setPower(0.0);
            }

            telemetry.addData("Status", "Stopped");
            telemetry.update();
            sleep(1000);
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

        frontLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        frontRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
        backLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        backRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
    }

    private void turnLeft(double degrees) {
        // Use perfected data: TURN_SPEED (0.33) and TURN_SECONDS (1.0 for 90 deg)
        frontLeftDrive.setPower(-TURN_SPEED);
        frontRightDrive.setPower(TURN_SPEED);
        backLeftDrive.setPower(-TURN_SPEED);
        backRightDrive.setPower(TURN_SPEED);
        
        sleep((long)((degrees / 90.0) * TURN_SECONDS * 1000));
        stopRobot();
        sleep(200);
    }

    private void turnRight(double degrees) {
        // Use perfected data: TURN_SPEED (0.33) and TURN_SECONDS (1.0 for 90 deg)
        frontLeftDrive.setPower(TURN_SPEED);
        frontRightDrive.setPower(-TURN_SPEED);
        backLeftDrive.setPower(TURN_SPEED);
        backRightDrive.setPower(-TURN_SPEED);

        sleep((long)((degrees / 90.0) * TURN_SECONDS * 1000));
        stopRobot();
        sleep(200);
    }

    private void stopRobot() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
