package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Autonomous OpMode named "secondaryparking".
 * Logic:
 * 1. Do nothing else - just move 60cm backward, then 180cm to the left.
 */
@Autonomous(name = "secondaryparking", group = "StarterBot")
public class SecondAuto extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.5;
    public static final double CM_PER_SECOND = 71.33; // 214cm / 3.0s

    // Calibration for straight line
    public static final double LEFT_P_SCALE = 0.959;
    public static final double RIGHT_P_SCALE = 1.00;

    // Ramping constants to fix jerk and drift
    public static final long RAMP_UP_TIME_MS = 500;
    public static final long RAMP_DOWN_TIME_MS = 500;

    // Compensation offsets
    public static final double START_COMPENSATION = 0.0;
    public static final double STOP_COMPENSATION = 0.08;
    // ----------------------------

    // Centralized hardware map shared across all OpModes
    private final RobotHardware robot = new RobotHardware();

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize all hardware devices using the shared hardware map
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Name: secondaryparking");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Move 60cm backward
            telemetry.addData("Status", "Moving 60cm backward");
            telemetry.update();

            moveBackward(60.0);
            stopRobot();
            sleep(250);

            // Move 180cm to the left
            telemetry.addData("Status", "Moving 180cm left");
            telemetry.update();

            moveLeft(180.0);
            stopRobot();

            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(500);
        }
    }

    private void moveBackward(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm backward", distanceCm);
        telemetry.update();

        // 1. Ramp up
        startMoving(-DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopMoving(-DRIVE_SPEED);
    }

    private void moveLeft(double distanceCm) {
        double timeSeconds = distanceCm / CM_PER_SECOND;

        telemetry.addData("Path", "Moving %.1f cm left", distanceCm);
        telemetry.update();

        // 1. Ramp up (negative strafe = left, per RobotHardware#driveMecanum convention)
        startStrafing(-DRIVE_SPEED);

        // 2. Constant speed
        long steadyStateTimeMs = (long) ((timeSeconds * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS);
        if (steadyStateTimeMs > 0) {
            sleep(steadyStateTimeMs);
        }

        // 3. Ramp down
        stopStrafing(-DRIVE_SPEED);
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

    private void setPowerWithSteer(double power, double steer) {
        double leftPower = (power - steer) * LEFT_P_SCALE;
        double rightPower = (power + steer) * RIGHT_P_SCALE;

        robot.frontLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        robot.frontRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
        robot.backLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        robot.backRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
    }

    private void stopRobot() {
        robot.stopDrive();
    }
}
