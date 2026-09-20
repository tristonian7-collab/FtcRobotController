package org.firstinspires.ftc.teamcode;

// Essential FTC SDK Imports for a Driving Chassis
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "MecanumStraferChassis", group = "Drive")
public class MecanumStraferChassis extends LinearOpMode {

    // Motor declarations for the 4-motor Mecanum chassis
    private DcMotor frontLeftDrive  = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive   = null;
    private DcMotor backRightDrive  = null;
    private CRServo feeder          = null;
    private DcMotor shooter         = null;
    private CRServo leftServo       = null;
    private CRServo rightServo      = null;

    // Speed multiplier (1.0 = 100% full speed capacity)
    private double maxDrivePower = 1.0;

    @Override
    public void runOpMode() {
        // Initialize hardware map (Ensure these names match your REV Hub configuration)
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        shooter         = hardwareMap.get(DcMotor.class, "shooterMotor");
        feeder          = hardwareMap.get(CRServo.class, "feederServo");
        leftServo       = hardwareMap.get(CRServo.class, "leftServo");
        rightServo      = hardwareMap.get(CRServo.class, "rightServo");

        // Reverse left side motors so positive power moves the robot forward
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        feeder.setDirection(CRServo.Direction.REVERSE);
        leftServo.setDirection(CRServo.Direction.FORWARD);
        rightServo.setDirection(CRServo.Direction.REVERSE);

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

            // 3. Mecanum Kinematics Power Distribution Formula
            double flPower = forward + turn + strafe;
            double frPower = forward - turn - strafe;
            double blPower = forward + turn - strafe;
            double brPower = forward - turn + strafe;

            // 4. Normalize powers if any calculated variable exceeds maximum limits (1.0 or -1.0)
            double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                    Math.max(Math.abs(blPower), Math.abs(brPower))));
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            // 5. Deliver proportional voltage straight to the REV Hub motor ports
            frontLeftDrive.setPower(flPower);
            frontRightDrive.setPower(frPower);
            backLeftDrive.setPower(blPower);
            backRightDrive.setPower(brPower);

            // 6. Control Servo and Feeder motor with R2 Trigger (gamepad1.right_trigger)
            feeder.setPower(gamepad1.right_trigger);
            leftServo.setPower(gamepad1.right_trigger);
            rightServo.setPower(gamepad1.right_trigger);

            // 7. Control Shooter motor with L2 Trigger (gamepad1.left_trigger)
            shooter.setPower(gamepad1.left_trigger);

            // 8. Monitor outputs live via driver station telemetry text feeds
            telemetry.addData("Joystick Inputs", "Y: (%.2f), X: (%.2f), Turn: (%.2f)", forward, strafe, turn);
            telemetry.addData("Motor Target Powers", "FL: (%.2f) | FR: (%.2f)", flPower, frPower);
            telemetry.addData("Motor Target Powers", "BL: (%.2f) | BR: (%.2f)", blPower, brPower);
            telemetry.addData("Servo + Feeder Power", "%.2f", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}
