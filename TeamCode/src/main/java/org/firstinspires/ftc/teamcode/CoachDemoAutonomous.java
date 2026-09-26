package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.List;

/**
 * Autonomous OpMode for Mecanum wheel chassis.
 * Logic:
 * 1. Moves forward until any April Tag is detected.
 * 2. Once a tag is detected, stop drive motors.
 * 3. Spin up the shooter for SHOOTER_SPINUP_SECONDS to build momentum.
 * 4. Then continuously run the feeder to fire balls, forever.
 */
@Autonomous(name = "CoachDemoAutonomous", group = "StarterBot")
public class CoachDemoAutonomous extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.25;
    public static final double SHOOTER_SPINUP_SECONDS = 1.5;
    // ----------------------------

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor shooter = null;
    private DcMotor feederMotor = null;
    private CRServo feederServo = null;
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

        try {
            feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");
        } catch (Exception e) {
            telemetry.addData("Warning", "feeder motor 'feederMotor' not found");
        }

        try {
            feederServo = hardwareMap.get(CRServo.class, "feederServo");
        } catch (Exception e) {
            telemetry.addData("Warning", "feeder servo 'feederServo' not found");
        }

        try {
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            rightServo = hardwareMap.get(CRServo.class, "rightServo");
        } catch (Exception e) {
            telemetry.addData("Warning", "Intake servos 'leftServo'/'rightServo' not found");
        }

        // Set directions (Matching MecanumStraferChassis configuration)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);

        if (feederMotor != null) {
            feederMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        if (feederServo != null) {
            feederServo.setDirection(CRServo.Direction.REVERSE);
        }
        if (leftServo != null) {
            leftServo.setDirection(CRServo.Direction.FORWARD);
        }
        if (rightServo != null) {
            rightServo.setDirection(CRServo.Direction.REVERSE);
        }

        // Set zero power behavior to BRAKE
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Vision
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "Initialized. Ready for Autonomous.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            boolean hasDetectedTag = false;

            // Phase 1: Drive forward until an AprilTag is detected. Feeder stays off.
            while (opModeIsActive() && !hasDetectedTag) {

                if (aprilTag != null) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    if (!currentDetections.isEmpty()) {
                        hasDetectedTag = true;
                    }
                }

                if (hasDetectedTag) {
                    // Tag found! Stop movement.
                    stopRobot();
                    telemetry.addData("Status", "AprilTag Detected! Stopping.");
                } else {
                    // No tag yet, continue moving forward
                    moveForward(DRIVE_SPEED);
                    telemetry.addData("Status", "Moving forward, searching for tags...");
                }

                telemetry.update();
                idle();
            }

            // Phase 2: Spin up the shooter first (feeder still off) so the ball
            // launches instead of just rolling on the shooter wheel.
            if (opModeIsActive()) {
                shooter.setPower(1.0);
                telemetry.addData("Status", "Spinning up shooter...");
                telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
                telemetry.update();

                ElapsedTime spinupTimer = new ElapsedTime();
                while (opModeIsActive() && spinupTimer.seconds() < SHOOTER_SPINUP_SECONDS) {
                    telemetry.addData("Status", "Spinning up shooter...");
                    telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
                    telemetry.update();
                    idle();
                }
            }

            // Phase 3: Shooter is up to speed, now continuously run the intake
            // group forever. leftServo, rightServo, feederMotor, and feederServo
            // must always run together.
            if (feederMotor != null) {
                feederMotor.setPower(1.0);
            }
            if (feederServo != null) {
                feederServo.setPower(1.0);
            }
            if (leftServo != null) {
                leftServo.setPower(1.0);
            }
            if (rightServo != null) {
                rightServo.setPower(1.0);
            }

            while (opModeIsActive()) {
                shooter.setPower(1.0);
                telemetry.addData("Status", "Shooter at speed. Feeding continuously...");
                telemetry.addData("Shooter Power", "%.2f", shooter.getPower());
                telemetry.update();
                idle();
            }
        }

        // Clean up vision portal resource when OpMode is done
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        aprilTag.setDecimation(1.0f);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void moveForward(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void stopRobot() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
