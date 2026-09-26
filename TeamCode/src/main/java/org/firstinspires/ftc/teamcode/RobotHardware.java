package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * Centralizes hardware device mapping, naming, and initialization for the robot so that
 * every OpMode (TeleOp and Autonomous) shares the exact same REV Hub configuration names,
 * motor/servo directions, and setup logic.
 * <p>
 * Usage: create one instance per OpMode and call {@link #init(HardwareMap)} at the top of
 * {@code runOpMode()}, then reference the public fields (or the helper methods below) instead
 * of mapping hardware manually.
 */
public class RobotHardware {

    // --- Drivetrain (4-motor Mecanum chassis) ---
    public DcMotor frontLeftDrive  = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor backRightDrive  = null;

    // --- Shooter / Intake Group ---
    // feederMotor rolls the ball into the robot (part of the intake group).
    // feederServo pushes the ball from intake into the shooter.
    // leftServo/rightServo are the intake rollers.
    // These four (leftServo, rightServo, feederMotor, feederServo) must always run together.
    public DcMotor shooter     = null;
    public DcMotor feederMotor = null;
    public CRServo feederServo = null;
    public CRServo leftServo   = null;
    public CRServo rightServo  = null;

    // --- Sensors ---
    public IMU imu = null;

    /**
     * Initializes all hardware devices using the standard REV Hub configuration names.
     * frontLeftDrive/frontRightDrive/backLeftDrive/backRightDrive/shooter/leftServo/rightServo
     * are required and will throw if missing from the configuration. feederMotor, feederServo,
     * and imu are optional (left null if not found/configured) so calling code should null-check
     * them before use.
     */
    public void init(HardwareMap hardwareMap) {
        // Required devices
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        shooter         = hardwareMap.get(DcMotor.class, "shooterMotor");
        leftServo       = hardwareMap.get(CRServo.class, "leftServo");
        rightServo      = hardwareMap.get(CRServo.class, "rightServo");

        // Optional devices
        try {
            feederServo = hardwareMap.get(CRServo.class, "feederServo");
        } catch (Exception e) {
            feederServo = null;
        }

        try {
            feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");
        } catch (Exception e) {
            feederMotor = null;
        }

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        } catch (Exception e) {
            imu = null;
        }

        // Directions (shared across all OpModes)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);
        if (feederServo != null) {
            feederServo.setDirection(CRServo.Direction.REVERSE);
        }
        if (feederMotor != null) {
            feederMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // Zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the 4-motor Mecanum chassis using standard forward/strafe/turn inputs,
     * normalizing power so no wheel exceeds +/-1.0.
     */
    public void driveMecanum(double forward, double strafe, double turn) {
        double flPower = forward + turn + strafe;
        double frPower = forward - turn - strafe;
        double blPower = forward + turn - strafe;
        double brPower = forward - turn + strafe;

        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    /** Immediately stops all drivetrain motors. */
    public void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    /**
     * Sets power for the intake group (leftServo, rightServo, feederMotor, feederServo).
     * These four components must always run together in sync.
     */
    public void setIntakePower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
        if (feederMotor != null) {
            feederMotor.setPower(power);
        }
        if (feederServo != null) {
            feederServo.setPower(power);
        }
    }

    /** Sets the flywheel (shooter) power. */
    public void setShooterPower(double power) {
        shooter.setPower(power);
    }
}
