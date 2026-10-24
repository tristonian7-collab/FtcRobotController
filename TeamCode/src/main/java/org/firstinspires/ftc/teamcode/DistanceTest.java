package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Distance Test", group = "Vision")
public class DistanceTest extends LinearOpMode {

    // --- Status Light PWM Positions ---
    private static final double STATUS_LIGHT_WHITE = 0.999;
    private static final double STATUS_LIGHT_GREEN = 0.500;

    // --- Hardware Members ---
    private Servo statusLight = null;

    // --- Vision Members ---
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;

    // --- Light PWM Control Members ---
    private double lightPwm = STATUS_LIGHT_WHITE;
    private boolean manualOverride = false;

    @Override
    public void runOpMode() {
        // Optional status light servo mapping
        try {
            statusLight = hardwareMap.get(Servo.class, "statusLight");
            if (statusLight != null) {
                statusLight.setPosition(STATUS_LIGHT_WHITE);
            }
        } catch (Exception e) {
            statusLight = null;
        }

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

        // Main detection and light control loop
        while (opModeIsActive()) {

            // AprilTag detection, distance reporting, and status light feedback
            boolean tagFound = false;
            boolean tagInRange = false;
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
                        // Apply distance calibration scale factor boosted by 2%
                        double rawRange = detection.ftcPose.range;
                        double range = rawRange * ((160.0 / 145.0) * 1.02);
                        telemetry.addData("Distance (Range)", "%.2f cm (Raw: %.2f)", range, rawRange);
                        telemetry.addData("Bearing", "%.2f degrees", detection.ftcPose.bearing);
                        telemetry.addData("Yaw", "%.2f degrees", detection.ftcPose.yaw);
                        telemetry.addData("Pitch", "%.2f degrees", detection.ftcPose.pitch);
                        telemetry.addData("Roll", "%.2f degrees", detection.ftcPose.roll);

                        if (range >= 140.0 && range <= 170.0) {
                            tagInRange = true;
                        }
                    } else {
                        telemetry.addData("Distance (Range)", "Tracking pose... (Hold Still)");
                    }
                    break;
                }
            } else {
                telemetry.addData("AprilTag Status", "Camera/Processor not initialized");
            }

            // --- Gamepad1 Right Joystick Light Control ---
            // Invert gamepad1.right_stick_y so pushing UP increases the PWM signal position
            double stickInput = -gamepad1.right_stick_y;
            if (Math.abs(stickInput) > 0.05) {
                manualOverride = true;
                lightPwm += stickInput * 0.003; // Adjust PWM signal position smoothly
                lightPwm = Range.clip(lightPwm, 0.0, 1.0);
            }

            // Pressing 'A' button on gamepad1 resets back to automatic AprilTag light mode
            if (gamepad1.a) {
                manualOverride = false;
            }

            // Determine light color / PWM signal based on manual override or AprilTag detection
            if (!manualOverride) {
                if (tagInRange) {
                    lightPwm = STATUS_LIGHT_GREEN;
                } else {
                    lightPwm = STATUS_LIGHT_WHITE;
                }
            }

            // Send PWM signal to the status light servo port
            if (statusLight != null) {
                statusLight.setPosition(lightPwm);
            }

            // Output Light PWM telemetry and status
            telemetry.addData("Light PWM Signal", "%.3f", lightPwm);
            if (manualOverride) {
                telemetry.addData("Status Light", "MANUAL (Right Stick Y: %.2f | Press 'A' to reset)", gamepad1.right_stick_y);
            } else {
                if (tagInRange) {
                    telemetry.addData("Status Light", "GREEN (Tag In Range: 140-160cm)");
                } else if (tagFound) {
                    telemetry.addData("Status Light", "WHITE (Tag Detected, Out of Range)");
                } else {
                    telemetry.addData("Status Light", "WHITE (No Tag)");
                }
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
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        // Force decimation to 1.0 for maximum shape sensitivity
        aprilTag.setDecimation(1.0f);

        // Create the vision portal manually with MJPEG streaming and continuous live view
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .addProcessor(aprilTag)
                .build();
    }
}
