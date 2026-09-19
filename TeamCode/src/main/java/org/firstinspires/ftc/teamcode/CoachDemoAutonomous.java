package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous OpMode for Mecanum wheel chassis.
 * Logic:
 * 1. Wait 2 seconds.
 * 2. Perform a square path 4 times.
 * 3. Each square consists of 4 segments: forward and then turn right.
 */
@Autonomous(name = "CoachDemoAutonomous", group = "StarterBot")
public class CoachDemoAutonomous extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED_SCALE = 5.0; // Speed 1-10
    public static final double FORWARD_SECONDS = 1.5;
    public static final double TURN_SECONDS = 0.8;      // Estimated time to turn 90 degrees
    // ----------------------------

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Set directions (Matching Teleop configuration)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to BRAKE for more precision
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double drivePower = DRIVE_SPEED_SCALE / 10.0;

        telemetry.addData("Status", "Initialized. Speed: %.1f", DRIVE_SPEED_SCALE);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // 1. Initial wait for 2 seconds
        telemetry.addData("Status", "Waiting for 2 seconds...");
        telemetry.update();
        sleep(2000);

        // Loop for 4 squares
        for (int squareCount = 1; squareCount <= 4 && opModeIsActive(); squareCount++) {
            
            // Each square has 4 sides/turns
            for (int sideCount = 1; sideCount <= 4 && opModeIsActive(); sideCount++) {
                
                // Update Telemetry
                telemetry.addData("Path", "Square %d of 4", squareCount);
                telemetry.addData("Side", "Segment %d of 4", sideCount);
                telemetry.update();

                // 2. Go Forward
                moveForward(drivePower);
                sleep((long)(FORWARD_SECONDS * 1000));
                
                // Stop before turning
                stopRobot();
                sleep(200);

                // 3. Turn Right
                turnRight(drivePower);
                sleep((long)(TURN_SECONDS * 1000));
                
                // Stop before next segment
                stopRobot();
                sleep(200);
            }
        }

        telemetry.addData("Status", "Completed 4 squares.");
        telemetry.update();
        sleep(2000);
    }

    private void moveForward(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void turnRight(double power) {
        // To turn right: Left wheels forward, Right wheels backward
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    private void stopRobot() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
