package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Angle Test", group = "Test")
public class servoAngleTest extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Initialize angle variables from current servo positions
        double nectarAngle = robot.getFeederServoPosition() * RobotHardware.FEEDER_SERVO_FULL_RANGE_DEGREES;
        double pollenAngle = robot.getPollenFeederServoPosition() * RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES;
        double rampAngle = robot.getRampServoPosition() * RobotHardware.RAMP_SERVO_FULL_RANGE_DEGREES;

        // Adjustment speed in degrees per loop iteration
        double angleStep = 0.5;

        telemetry.addData("Status", "Servo Angle Test Initialized. Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Nectar Feeder Servo: Left Trigger decreases, Right Trigger increases
            double triggerDelta = gamepad1.right_trigger - gamepad1.left_trigger;
            nectarAngle += triggerDelta * angleStep;

            // Pollen Feeder Servo: Left Bumper decreases, Right Bumper increases
            if (gamepad1.left_bumper) {
                pollenAngle -= angleStep;
            }
            if (gamepad1.right_bumper) {
                pollenAngle += angleStep;
            }

            // Ramp Servo: Square decreases, Triangle increases
            if (gamepad1.square) {
                rampAngle -= angleStep;
            }
            if (gamepad1.triangle) {
                rampAngle += angleStep;
            }

            // Clamp angles to valid physical ranges [0, FULL_RANGE_DEGREES]
            nectarAngle = Range.clip(nectarAngle, 0.0, RobotHardware.FEEDER_SERVO_FULL_RANGE_DEGREES);
            pollenAngle = Range.clip(pollenAngle, 0.0, RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES);
            rampAngle = Range.clip(rampAngle, 0.0, RobotHardware.RAMP_SERVO_FULL_RANGE_DEGREES);

            // Command the robot hardware
            robot.setFeederServoPosition(RobotHardware.feederServoDegreesToPosition(nectarAngle));
            robot.setPollenFeederServoPosition(pollenAngle / RobotHardware.POLLEN_FEEDER_SERVO_FULL_RANGE_DEGREES);
            robot.setRampServoPosition(RobotHardware.rampServoDegreesToPosition(rampAngle));

            // Telemetry reporting
            telemetry.addData("Nectar Feeder Servo", "Angle: %.1f° | Pos: %.3f", nectarAngle, robot.getFeederServoPosition());
            telemetry.addData("Pollen Feeder Servo", "Angle: %.1f° | Pos: %.3f", pollenAngle, robot.getPollenFeederServoPosition());
            telemetry.addData("Ramp Servo", "Angle: %.1f° | Pos: %.3f", rampAngle, robot.getRampServoPosition());
            telemetry.addData("Controls", "L/R Trigger = Nectar | L/R Bumper = Pollen | Square/Triangle = Ramp");

            telemetry.update();
            sleep(20); // Small delay for smooth adjustment
        }
    }
}
