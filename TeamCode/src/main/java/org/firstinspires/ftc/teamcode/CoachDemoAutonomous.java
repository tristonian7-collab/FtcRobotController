package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
                robot.setShooterPower(0.4);
                telemetry.addData("Status", "Spinning up shooter...");
                telemetry.addData("Shooter Power", "%.2f", robot.shooter.getPower());
                telemetry.update();

                ElapsedTime spinupTimer = new ElapsedTime();
                while (opModeIsActive() && spinupTimer.seconds() < SHOOTER_SPINUP_SECONDS) {
                    telemetry.addData("Status", "Spinning up shooter...");
                    telemetry.addData("Shooter Power", "%.2f", robot.shooter.getPower());
                    telemetry.update();
                    idle();
                }
            }

            // Phase 3: Shooter is up to speed, now continuously run the intake
            // group forever. leftServo, rightServo, feederMotor, and feederServo
            // must always run together.
            robot.setIntakePower(1.0);

            while (opModeIsActive()) {
                robot.setShooterPower(0.4);
                telemetry.addData("Status", "Shooter at speed. Feeding continuously...");
                telemetry.addData("Shooter Power", "%.2f", robot.shooter.getPower());
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
        robot.driveMecanum(power, 0, 0);
    }

    private void stopRobot() {
        robot.stopDrive();
    }
}
