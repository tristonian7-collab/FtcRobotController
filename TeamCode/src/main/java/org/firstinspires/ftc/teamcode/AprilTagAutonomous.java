package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@Autonomous(name="AprilTag Auto Firing", group="Robot")
public class AprilTagAutonomous extends LinearOpMode {

    /* Hardware members */
    private DcMotor leftDrive   = null;
    private DcMotor rightDrive  = null;
    private DcMotor shooter     = null;
    private Servo   feeder      = null;
    private IMU     imu         = null;

    /* Vision members */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /* Constants */
    final double DESIRED_DISTANCE = 24.0; // inches
    final double SPEED_GAIN = 0.02;
    final double TURN_GAIN  = 0.01;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    final double SHOOTER_POWER = 0.8;
    final double FEEDER_FIRE_POS = 0.5;
    final double FEEDER_IDLE_POS = 0.0;

    @Override
    public void runOpMode() {
        // Initialize Hardware
        try {
            leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            shooter    = hardwareMap.get(DcMotor.class, "shooter");
            feeder     = hardwareMap.get(Servo.class, "feederServo");

            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            shooter.setDirection(DcMotor.Direction.FORWARD);

            feeder.setPosition(FEEDER_IDLE_POS);

            // Initialize IMU
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));

        } catch (Exception e) {
            telemetry.addData("Error", "Hardware not found. Check configuration.");
            telemetry.update();
        }

        // Initialize Vision
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Error", "Vision Portal failed to initialize.");
            telemetry.update();
        }

        telemetry.addData("Status", "Initialized. Ready for Start.");
        telemetry.update();

        // Loop while waiting for start to show "all the time" telemetry
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized. Ready for Start.");
            telemetry.addData("Y Controller Angle (Stick)", gamepad1.left_stick_y);
            
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Y Controller Angle (Pitch)", orientation.getPitch(AngleUnit.DEGREES));
            
            telemetry.update();
            sleep(10);
        }

        // Start a timer for the autonomous period (30 seconds)
        double startTime = getRuntime();
        double lastFoundTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime < 30)) {
            AprilTagDetection targetTag = findTag(-1); // Search for any tag

            if (targetTag != null) {
                lastFoundTime = getRuntime();
                // Determine errors
                double rangeError = (targetTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = targetTag.ftcPose.bearing;

                // Check if aligned and at distance (with a small tolerance)
                if (Math.abs(rangeError) < 1.0 && Math.abs(headingError) < 2.0) {
                    // Stop and Fire
                    moveRobot(0, 0);
                    fireShooter();
                    break; // End autonomous after firing
                } else {
                    // Drive to target
                    // We want to drive forward/back and rotate to face the tag
                    double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    
                    telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
                    moveRobot(drive, turn);
                }

                if (targetTag instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) targetTag;
                    telemetry.addData("Target Found", "ID %d (%s)", singleDet.id, singleDet.metadata.name);
                } else {
                    telemetry.addData("Target Found", "Unknown AprilTag structure");
                }
                telemetry.addData("Range", "%5.1f inches", targetTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", targetTag.ftcPose.bearing);
            } else {
                // No tag found
                if (getRuntime() - lastFoundTime > 2.0) {
                    // Scan by rotating slowly
                    moveRobot(0, 0.2);
                    telemetry.addData("Status", "Scanning...");
                } else {
                    moveRobot(0, 0);
                    telemetry.addData("Status", "Searching for Tag...");
                }
            }
            telemetry.addData("Y Controller Angle (Stick)", gamepad1.left_stick_y);
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Y Controller Angle (Pitch)", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.update();
            sleep(10);
        }

        // Cleanup
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private AprilTagDetection findTag(int targetId) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                if (singleDet.metadata != null) {
                    if (targetId < 0 || singleDet.id == targetId) {
                        return detection;
                    }
                }
            }
        }
        return null;
    }

    private void moveRobot(double x, double yaw) {
        double leftPower  = x - yaw;
        double rightPower = x + yaw;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private void fireShooter() {
        telemetry.addData("Action", "Firing!!!");
        telemetry.update();

        shooter.setPower(SHOOTER_POWER);
        sleep(2000); // Wait for shooter to spin up

        feeder.setPosition(FEEDER_FIRE_POS);
        sleep(1000); // Wait for feed

        feeder.setPosition(FEEDER_IDLE_POS);
        shooter.setPower(0);
    }
}
