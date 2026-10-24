package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous OpMode named "secondaryparking2".
 * Logic:
 * 1. Do nothing else - just move 60cm to the right.
 */
@Autonomous(name = "secondaryparking2", group = "StarterBot")
public class CornerAuto extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.5;
    public static final double CM_PER_SECOND = 71.33; // 214cm / 3.0s

    // Ramping constants to fix jerk and drift
    public static final long RAMP_UP_TIME_MS = 500;
    public static final long RAMP_DOWN_TIME_MS = 500;
    // ----------------------------

    // Centralized hardware map shared across all OpModes
    private final RobotHardware robot = new RobotHardware();

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize all hardware devices using the shared hardware map
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Name: secondaryparking2");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Move 60cm to the right
            telemetry.addData("Status", "Moving 60cm right");
            telemetry.update();

            moveRight(60.0);
            stopRobot();

            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(500);
        }
    }

    private void moveRight(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm right", distanceCm);
        telemetry.update();

        // 1. Ramp up (positive strafe = right, per RobotHardware#driveMecanum convention)
        startStrafing(DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopStrafing(DRIVE_SPEED);
    }

    private void startStrafing(double targetStrafePower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_UP_TIME_MS) {
            double ratio = rampTimer.milliseconds() / RAMP_UP_TIME_MS;
            double currentPower = targetStrafePower * ratio;
            robot.driveMecanum(0.0, currentPower, 0.0);
        }
        robot.driveMecanum(0.0, targetStrafePower, 0.0);
    }

    private void stopStrafing(double startingPower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_DOWN_TIME_MS) {
            double ratio = 1.0 - (rampTimer.milliseconds() / RAMP_DOWN_TIME_MS);
            double currentPower = startingPower * ratio;
            robot.driveMecanum(0.0, currentPower, 0.0);
        }
        stopRobot();
    }

    private void stopRobot() {
        robot.stopDrive();
    }
}
