package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.List;

/**
 * Autonomous OpMode named "autoHACC".
 * Displays AprilTag tracking data onto the Driver Hub telemetry layout.
 */
@Autonomous(name = "autoHACC", group = "StarterBot")
public class AutoHACC extends LinearOpMode {

    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize Vision
        try {
            aprilTag = new AprilTagProcessor.Builder().build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Webcam 1 not found or failed to initialize.");
        }

        telemetry.addData("Status", "Initialized. Name: autoHACC");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running. Time elapsed: %.1f seconds", runtime.seconds());

            if (aprilTag != null) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                boolean tagFound = false;

                for (AprilTagDetection detection : currentDetections) {
                    tagFound = true;
                    
                    if (detection instanceof AprilTagSingleDetection) {
                        AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                        String tagName = (singleDet.metadata != null) ? singleDet.metadata.name : "Unknown Tag";
                        telemetry.addData("AprilTag State", "VISIBLE");
                        telemetry.addData("Tag Found", "ID %d (%s)", singleDet.id, tagName);
                    } else {
                        telemetry.addData("AprilTag State", "VISIBLE");
                        telemetry.addData("Tag Found", "Tag Cluster");
                    }
                    
                    telemetry.addData("Distance (Range)", "%.2f inches", detection.ftcPose.range);
                    telemetry.addData("Bearing (Angle)", "%.2f degrees", detection.ftcPose.bearing);
                    break; // Display the primary/first detected tag
                }

                if (!tagFound) {
                    telemetry.addData("AprilTag State", "NOT VISIBLE");
                }
            } else {
                telemetry.addData("AprilTag Status", "Processor Offline");
            }

            telemetry.update();
            sleep(50); // Small delay to keep the layout readable
        }

        // Clean up resources on stop
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
