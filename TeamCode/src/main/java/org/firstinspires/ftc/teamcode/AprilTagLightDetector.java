package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Light Detector", group = "Vision")
public class AprilTagLightDetector extends LinearOpMode {

    // Centralized hardware map shared across all OpModes
    private final RobotHardware robot = new RobotHardware();

    // --- Vision Members ---
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;

    @Override
    public void runOpMode() {
        // Initialize hardware devices (status light, etc.) using the shared hardware map
        robot.init(hardwareMap);
        robot.setStatusLightWhite(); // Default status light to white

        // Initialize Vision (try-catch since camera may not be mounted/configured yet)
        try {
            initAprilTag();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "AprilTag Light Detector Initialized. Ready!");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station app
        waitForStart();

        // Main detection loop
        while (opModeIsActive()) {

            // AprilTag detection, distance reporting, and status light feedback
            boolean tagFound = false;
            if (aprilTag != null) {
                telemetry.addData("Camera State", visionPortal.getCameraState());

                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("Raw Detections Count", currentDetections.size());
                for (AprilTagDetection detection : currentDetections) {
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
                        telemetry.addData("Yaw", "%.2f degrees", detection.ftcPose.yaw);
                        telemetry.addData("Pitch", "%.2f degrees", detection.ftcPose.pitch);
                        telemetry.addData("Roll", "%.2f degrees", detection.ftcPose.roll);
                    } else {
                        telemetry.addData("Distance (Range)", "Tracking pose... (Hold Still)");
                    }
                    break;
                }
            } else {
                telemetry.addData("AprilTag Status", "Camera/Processor not initialized");
            }

            // Update status light: white when no tag detected, green when tag detected
            if (tagFound) {
                robot.setStatusLightGreen();
                telemetry.addData("Status Light", "GREEN (Tag Detected)");
            } else {
                robot.setStatusLightWhite();
                telemetry.addData("Status Light", "WHITE (No Tag)");
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
