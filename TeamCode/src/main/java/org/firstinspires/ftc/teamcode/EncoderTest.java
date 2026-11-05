package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderTest extends OpMode {
    RobotHardware test = new RobotHardware();

    @Override
    public void init() {
        test.init(hardwareMap);
    }

    @Override
    public void loop() {
        test.setShooterPower(0.5);
        telemetry.addData("Motor Revs", test.getMotorRevs());
    }
}
