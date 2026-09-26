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
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
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
    private CRServo feederServo    = null;
    private DcMotor feederMotor    = null;
    private DcMotor shooter        = null;
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
        shooter = hardwareMap.get(DcMotor.class, "shooterMotor");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");

        try {
            feederServo = hardwareMap.get(CRServo.class, "feederServo");
        } catch (Exception e) {
            telemetry.addData("Warning", "feeder servo 'feederServo' not found");
        }

        try {
            feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");
        } catch (Exception e) {
            telemetry.addData("Warning", "feeder motor 'feederMotor' not found");
        }

        // Set directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        if (feederServo != null) {
            feederServo.setDirection(CRServo.Direction.REVERSE);
        }
        if (feederMotor != null) {
            feederMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        shooter.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);



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
            // Spin the intake group (leftServo, rightServo, feederMotor, feederServo)
            // continuously for the entire OpMode. These must always run together.
            if (feederMotor != null) {
                feederMotor.setPower(1.0);
            }
            if (feederServo != null) {
                feederServo.setPower(1.0);
            }
            // Flywheel (shooter) spins continuously at half power for the entire OpMode.
            if (shooter != null) {
                shooter.setPower(1.0);
            }

            // Run for 10 seconds or until STOP is pressed
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
            
            while (opModeIsActive() && timer.seconds() < 10) {
                // Intake group: leftServo, rightServo, feederMotor, feederServo always together
                leftServo.setPower(1.0);
                rightServo.setPower(1.0);

                if (feederMotor != null) {
                    feederMotor.setPower(1.0);
                }
                if (feederServo != null) {
                    feederServo.setPower(1.0);
                }
                // Flywheel remains spinning continuously at half power
                if (shooter != null) {
                    shooter.setPower(1.0);
                }

                // Show what the software thinks is happening
                telemetry.addData("Status", "feeder Running");
                telemetry.addData("Timer", "%.1f / 10.0s", timer.seconds());
                telemetry.addData("Servo L", "Power: %.2f", leftServo.getPower());
                telemetry.addData("Servo R", "Power: %.2f", rightServo.getPower());
                if (feederServo != null) {
                    telemetry.addData("feeder Servo", "Power: %.2f", feederServo.getPower());
                } else {
                    telemetry.addData("feeder Servo", "NOT FOUND");
                }
                if (feederMotor != null) {
                    telemetry.addData("feeder Motor", "Power: %.2f", feederMotor.getPower());
                } else {
                    telemetry.addData("feeder Motor", "NOT FOUND");
                }
                if (shooter != null) {
                    telemetry.addData("shooter Motor", "Power: %.2f", shooter.getPower());
                } else {
                    telemetry.addData("shooter Motor", "NOT FOUND");
                }

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
                        
                        // Flywheel is already spinning continuously at half power
                        if (shooter != null) {
                            telemetry.addData("Shooter Status", "Already spinning at half power");
                            telemetry.update();
                        }
                        
                        // Break out of the loop completely once a tag is found to prevent it from starting again
                        break; 
                    } else {
                        // No tag in sight yet, continue moving forward at a safe testing speed
                        telemetry.addData("AprilTag Status", "No tags visible - Moving Forward...");
                        setPowerWithSteer(0.25, 0.0);
                    }
                } else {
                    telemetry.addData("AprilTag Status", "Camera/Processor not initialized (not mounted or configured yet)");
                }

                telemetry.update();
                
                idle();
            }

            // Stop everything. Intake group (leftServo, rightServo, feederMotor,
            // feederServo) always stops together since they must run in sync.
            stopRobot();
            leftServo.setPower(0.0);
            rightServo.setPower(0.0);
            if (feederMotor != null) {
                feederMotor.setPower(0.0);
            }
            if (feederServo != null) {
                feederServo.setPower(0.0);
            }
            if (shooter != null) {
                shooter.setPower(0.0);
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
