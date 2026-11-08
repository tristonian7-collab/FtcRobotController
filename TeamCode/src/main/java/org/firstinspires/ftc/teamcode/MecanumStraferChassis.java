package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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

    // --- Light PWM Control Members ---
    private double lightPwm = RobotHardware.STATUS_LIGHT_WHITE;

    // --- Status Light INIT color cycle (purely cosmetic/diagnostic; run-time colors below are untouched) ---
    // Sweeps across the servo's full range (plus the named RED/GREEN constants) so the light visibly
    // cycles while waiting at INIT, confirming it's wired/mapped correctly before PLAY is pressed.
    private static final double[] INIT_CYCLE_POSITIONS = {
            0.0,
            0.2,
            RobotHardware.STATUS_LIGHT_RED,
            0.4,
            0.6,
            RobotHardware.STATUS_LIGHT_GREEN,
            0.8,
            1.0,
    };
    private static final double INIT_CYCLE_SECONDS_PER_COLOR = 0.5;

    // --- Gate Servo Momentary Hold Controls (Triangle = feederServo, Square = pollenFeederServo) ---
    // Both servos are ranged (positional) servos, so "up"/"down" are absolute positions that are
    // commanded directly every loop based on whether the button is currently held; the servo's
    // own hardware handles physically moving there. Triangle: held = up, released = down.
    // pollenFeederServo rests at 120 degrees when square is released, and moves down to 20
    // degrees for as long as square is held, snapping back to 120 degrees the instant it's
    // released. There is no toggle/latching behavior and no edge-detection needed.
    // TODO: Calibrate FEEDER_SERVO_FULL_RANGE_DEGREES / POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES (in
    // RobotHardware) to each servo's actual physical range of motion so these degree positions
    // are accurate. If a direction is backwards on the robot, flip that servo's direction in
    // RobotHardware#init instead.
    private static final double FEEDER_SERVO_ROTATE_DEGREES = 120.0;
    private static final double FEEDER_SERVO_BOTTOM_POSITION = 0.0;
    private static final double FEEDER_SERVO_UP_POSITION =
            FEEDER_SERVO_BOTTOM_POSITION
                    + (FEEDER_SERVO_ROTATE_DEGREES / RobotHardware.FEEDER_SERVO_FULL_RANGE_DEGREES);

    // pollenFeederServo: released (resting) = 120 degrees, held (square pressed) = 20 degrees.
    private static final double POLLEN_FEEDER_SERVO_RESTING_DEGREES = 120.0;
    private static final double POLLEN_FEEDER_SERVO_PRESSED_DEGREES = 20.0;
    private static final double POLLEN_FEEDER_SERVO_BOTTOM_POSITION =
            POLLEN_FEEDER_SERVO_RESTING_DEGREES / RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES;
    private static final double POLLEN_FEEDER_SERVO_UP_POSITION =
            POLLEN_FEEDER_SERVO_PRESSED_DEGREES / RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES;

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

        // Cycle the status light through a spread of colors while waiting at INIT so it's
        // obvious the light is wired/mapped correctly before PLAY is pressed. Run-time status
        // light behavior (red/green based on AprilTag detection) is untouched below.
        ElapsedTime initColorTimer = new ElapsedTime();
        int initColorIndex = 0;
        while (!isStarted() && !isStopRequested()) {
            if (initColorTimer.seconds() >= INIT_CYCLE_SECONDS_PER_COLOR) {
                initColorIndex = (initColorIndex + 1) % INIT_CYCLE_POSITIONS.length;
                initColorTimer.reset();
            }
            robot.setStatusLightColor(INIT_CYCLE_POSITIONS[initColorIndex]);

            telemetry.addData("Status", "Chassis Initialized. Cycling status light. Ready to drive!");
            telemetry.update();
        }

        // Wait for the driver to press PLAY on the Driver Station app
        waitForStart();
        robot.setStatusLightWhite(); // Default status light to white once run-time logic takes over

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

            // Triangle holds feederServo (gate) up for as long as it's held, and releases it back
            // down the instant triangle is let go. Square controls pollenFeederServo: it rests at
            // 120 degrees and moves down to 20 degrees for as long as square is held, snapping
            // back to 120 degrees the instant square is released. The left trigger still acts as
            // a manual override for feederServo, directly setting its absolute position when
            // pressed (takes priority over triangle).
            boolean trianglePressed = gamepad1.triangle;
            boolean squarePressed = gamepad1.square;
            telemetry.addData("DEBUG gamepad1.square (raw)", squarePressed);
            telemetry.addData("DEBUG gamepad1.cross/x (raw)", gamepad1.x);

            double feederServoPosition;
            if (gamepad1.left_trigger > 0.05) {
                // Manual override: left trigger directly sets the servo's absolute position.
                feederServoPosition = gamepad1.left_trigger;
            } else {
                feederServoPosition = trianglePressed ? FEEDER_SERVO_UP_POSITION : FEEDER_SERVO_BOTTOM_POSITION;
            }
            robot.setFeederServoPosition(feederServoPosition);

            // pollenFeederServo rests at 120 degrees (POLLEN_FEEDER_SERVO_BOTTOM_POSITION) when
            // square is released, and holding square drives it down to 20 degrees
            // (POLLEN_FEEDER_SERVO_UP_POSITION) for as long as it's held.
            double pollenFeederServoPosition =
                    squarePressed ? POLLEN_FEEDER_SERVO_UP_POSITION : POLLEN_FEEDER_SERVO_BOTTOM_POSITION;
            robot.setPollenFeederServoPosition(pollenFeederServoPosition);

            robot.setTargetShooterRPM(25);

            // 8. Monitor outputs live via driver station telemetry text feeds
            telemetry.addData("Joystick Inputs", "Y: (%.2f), X: (%.2f), Turn: (%.2f)", forward, strafe, turn);
            telemetry.addData("Motor Target Powers", "FL: (%.2f) | FR: (%.2f)", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
            telemetry.addData("Motor Target Powers", "BL: (%.2f) | BR: (%.2f)", robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
            telemetry.addData("Intake Group Power (L/R Servo, Feeder Motor/Servo)", "%.2f", intakePower);
            telemetry.addData("Feeder Servo Position (Commanded)", "%.2f", feederServoPosition);
            telemetry.addData("Feeder Servo Position (Actual)", "%.2f", robot.feederServo != null ? robot.feederServo.getPosition() : -1.0);
            telemetry.addData("Feeder Servo Angle (Triangle Held)", "%.1f degrees", feederServoPosition * RobotHardware.FEEDER_SERVO_FULL_RANGE_DEGREES);
            telemetry.addData("Pollen Feeder Servo Position (Commanded)", "%.2f", pollenFeederServoPosition);
            telemetry.addData("Pollen Feeder Servo Position (Actual)", "%.2f", robot.pollenFeederServo != null ? robot.pollenFeederServo.getPosition() : -1.0);
            telemetry.addData("Pollen Feeder Servo Angle (Square Held)", "%.1f degrees", pollenFeederServoPosition * RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES);
            telemetry.addData("Target RPM: ", robot.getTargetShooterRPM());
            telemetry.addData("RPMs: ", robot.getMotorRPM());

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

            // Determine light color / PWM signal based on AprilTag detection (no manual override)
            if (tagFound) {
                lightPwm = RobotHardware.STATUS_LIGHT_GREEN;
            } else {
                lightPwm = RobotHardware.STATUS_LIGHT_RED;
            }

            // Send PWM signal to the status light servo port
            robot.setStatusLightColor(lightPwm);

            // Output Light PWM telemetry and status
            telemetry.addData("Light PWM Signal", "%.3f", lightPwm);
            telemetry.addData("Status Light", tagFound ? "GREEN (Tag Detected)" : "RED (No Tag)");

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
