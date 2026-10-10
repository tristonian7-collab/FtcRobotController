package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "MecanumStraferChassis", group = "Drive")
public class MecanumStraferChassis extends LinearOpMode {

    // Centralized hardware map shared across all OpModes
    private final RobotHardware robot = new RobotHardware();

    // Speed multiplier (1.0 = 100% full speed capacity)
    private double maxDrivePower = 1.0;

    // --- Vision Members ---
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;

    @Override
    public void runOpMode() {
        // Initialize all hardware devices using the shared hardware map
        robot.init(hardwareMap);

        // Initialize Vision (try-catch since camera may not be mounted/configured yet)
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "Chassis Initialized. Ready to drive!");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station app
        waitForStart();

        // Main driver loop
        while (opModeIsActive()) {

            // 1. Gather Joystick Inputs from Gamepad 1
            // Invert left_stick_y because pushing the stick up naturally reads negative
            double forward = -gamepad1.left_stick_y;
            double strafe  = gamepad1.left_stick_x;
            double turn    = gamepad1.right_stick_x;

            // 2. Apply the Max Speed Restriction Modifier
            forward = forward * maxDrivePower;
            strafe  = strafe * maxDrivePower;
            turn    = turn * maxDrivePower;

            // 3-5. Mecanum Kinematics Power Distribution, normalization, and motor output
            robot.driveMecanum(forward, strafe, turn);

            // 6. Control Intake Group (leftServo, rightServo, feederMotor, feederServo) together with R2 Trigger
            double intakePower = gamepad1.right_trigger;
            robot.setIntakePower(intakePower);

            double feederServoPower = gamepad1.left_trigger;
            robot.setFeederServo(feederServoPower);

            // 7. Flywheel (shooter) always spins at a fixed power; L2 trigger no longer controls it
            robot.setShooterPower(0.5);

            // 8. Monitor outputs live via driver station telemetry text feeds
            telemetry.addData("Joystick Inputs", "Y: (%.2f), X: (%.2f), Turn: (%.2f)", forward, strafe, turn);
            telemetry.addData("Motor Target Powers", "FL: (%.2f) | FR: (%.2f)", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
            telemetry.addData("Motor Target Powers", "BL: (%.2f) | BR: (%.2f)", robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
            telemetry.addData("Intake Group Power (L/R Servo, Feeder Motor/Servo)", "%.2f", intakePower);
            telemetry.addData("Shooter Power", "%.2f", robot.shooter.getPower());

            // AprilTag detection and distance reporting
            if (aprilTag != null) {
                telemetry.addData("Camera State", visionPortal.getCameraState());

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("Raw Detections Count", currentDetections.size());
                for (AprilTagDetection detection : currentDetections) {
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
            } else {
                telemetry.addData("AprilTag Status", "Camera/Processor not initialized");
            }

            telemetry.update();
        }

        // Clean up vision portal resource when OpMode is done
        if (visionPortal != null) {
            visionPortal.close();
        }
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
