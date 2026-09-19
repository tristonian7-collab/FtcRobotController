package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Autonomous OpMode named "autotest".
 * Used to calibrate straight line movement, start jerks, and stop drifts.
 */
@Autonomous(name = "autotest", group = "StarterBot")
public class Autotest extends LinearOpMode {

    // --- Adjustable Constants ---
    public static final double DRIVE_SPEED = 0.5;
    public static final double FORWARD_SECONDS = 3.0;
    
    // Power scaling for straight line (from SkongAutonomous)
    public static final double LEFT_P_SCALE = 0.959; 
    public static final double RIGHT_P_SCALE = 1.00;

    // Ramping constants to fix jerk and drift
    public static final long RAMP_UP_TIME_MS = 500;
    public static final long RAMP_DOWN_TIME_MS = 500;
    
    // Compensation offsets (adjust these if ramping isn't enough)
    // Positive = steer left, Negative = steer right
    public static final double START_COMPENSATION = 0.0; 
    public static final double STOP_COMPENSATION = 0.08;
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

        // Set directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior to BRAKE
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Calibrating Straight Line...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Starting Move...");
            telemetry.update();
            
            // Start with compensation
            startMoving(DRIVE_SPEED);
            
            // Hold full speed
            telemetry.addData("Status", "Full Speed Move");
            telemetry.update();
            sleep((long)((FORWARD_SECONDS * 1000) - RAMP_UP_TIME_MS - RAMP_DOWN_TIME_MS));
            
            // Stop with compensation
            telemetry.addData("Status", "Stopping Move...");
            telemetry.update();
            stopMoving(DRIVE_SPEED);
            
            telemetry.addData("Status", "Done");
            telemetry.update();
            sleep(2000);
        }
    }

    /**
     * Ramps up power to target to avoid starting jerk.
     */
    private void startMoving(double targetPower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_UP_TIME_MS) {
            double ratio = rampTimer.milliseconds() / RAMP_UP_TIME_MS;
            double currentPower = targetPower * ratio;
            
            // Apply straight-line scaling + start compensation (fade out)
            setPowerWithSteer(currentPower, START_COMPENSATION * ratio);
        }
        // Ensure we end at full target power
        setPowerWithSteer(targetPower, 0);
    }

    /**
     * Ramps down power to zero to avoid stopping drift.
     */
    private void stopMoving(double startingPower) {
        ElapsedTime rampTimer = new ElapsedTime();
        while (opModeIsActive() && rampTimer.milliseconds() < RAMP_DOWN_TIME_MS) {
            double ratio = 1.0 - (rampTimer.milliseconds() / RAMP_DOWN_TIME_MS);
            double currentPower = startingPower * ratio;
            
            // Apply straight-line scaling + stop compensation (fade out with power)
            setPowerWithSteer(currentPower, STOP_COMPENSATION * ratio);
        }
        // Full stop
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void setPowerWithSteer(double power, double steer) {
        double leftPower = (power - steer) * LEFT_P_SCALE;
        double rightPower = (power + steer) * RIGHT_P_SCALE;
        
        frontLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        frontRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
        backLeftDrive.setPower(Range.clip(leftPower, -1.0, 1.0));
        backRightDrive.setPower(Range.clip(rightPower, -1.0, 1.0));
    }
}
