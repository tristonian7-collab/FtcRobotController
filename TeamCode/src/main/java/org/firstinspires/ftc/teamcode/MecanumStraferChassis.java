package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MecanumStraferChassis", group = "Drive")
public class MecanumStraferChassis extends LinearOpMode {

    // Centralized hardware map shared across all OpModes
    private final RobotHardware robot = new RobotHardware();

    // Speed multiplier (1.0 = 100% full speed capacity)
    private double maxDrivePower = 1.0;

    @Override
    public void runOpMode() {
        // Initialize all hardware devices using the shared hardware map
        robot.init(hardwareMap);

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

            // 3-5. Mecanum Kinematics Power Distribution, normalization, and motor output
            robot.driveMecanum(forward, strafe, turn);

            // 6. Control Intake Group (leftServo, rightServo, feederMotor, feederServo) together with R2 Trigger
            double intakePower = gamepad1.right_trigger;
            robot.setIntakePower(intakePower);

            // 7. Flywheel (shooter) always spins at a fixed power; L2 trigger no longer controls it
            robot.setShooterPower(0.4);

            // 8. Monitor outputs live via driver station telemetry text feeds
            telemetry.addData("Joystick Inputs", "Y: (%.2f), X: (%.2f), Turn: (%.2f)", forward, strafe, turn);
            telemetry.addData("Motor Target Powers", "FL: (%.2f) | FR: (%.2f)", robot.frontLeftDrive.getPower(), robot.frontRightDrive.getPower());
            telemetry.addData("Motor Target Powers", "BL: (%.2f) | BR: (%.2f)", robot.backLeftDrive.getPower(), robot.backRightDrive.getPower());
            telemetry.addData("Intake Group Power (L/R Servo, Feeder Motor/Servo)", "%.2f", intakePower);
            telemetry.addData("Shooter Power", "%.2f", robot.shooter.getPower());
            telemetry.update();
        }
    }
}
