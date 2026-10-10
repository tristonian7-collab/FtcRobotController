package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Standalone diagnostic OpMode for the status light ONLY.
 * <p>
 * This intentionally goes through {@code RobotHardware.init(...)} (same as every other
 * OpMode) instead of mapping the servo directly, so that if the light doesn't light up here it
 * tells us the problem is in the hardware configuration / mapping (wrong name, wrong port, bad
 * wiring) and not in the RobotHardware class itself or in any other OpMode's logic.
 * <p>
 * As soon as INIT is pressed on the Driver Station, this repeatedly cycles the status light
 * through a spread of servo positions (including the known RED/GREEN constants from
 * RobotHardware plus several points across the full 0.0-1.0 range) so you can visually confirm
 * the light is responding without even needing to press START.
 */
@TeleOp(name = "Status Light Test", group = "Test")
public class StatusLightTest extends LinearOpMode {

    // Centralized hardware map - same class every other OpMode uses
    private final RobotHardware robot = new RobotHardware();

    // How long to hold each color before advancing to the next one
    private static final double SECONDS_PER_COLOR = 1.0;

    // Sweep across the full servo range plus the two known named colors so we can see
    // whatever the light is actually capable of, not just red/green.
    private static final double[] TEST_POSITIONS = {
            0.0,
            0.2,
            RobotHardware.STATUS_LIGHT_RED,
            0.4,
            0.6,
            RobotHardware.STATUS_LIGHT_GREEN,
            0.8,
            1.0,
    };

    @Override
    public void runOpMode() {
        // Initialize all hardware devices using the shared hardware map, exactly like every
        // other OpMode. If "statusLight" isn't configured with this exact name on the REV Hub,
        // robot.statusLight will be null and setStatusLightColor() below will silently no-op -
        // the telemetry line makes that obvious instead of leaving you guessing.
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - cycling status light colors now");
        telemetry.addData("statusLight hardware mapped?",
                robot.statusLight != null ? "YES" : "NO - check REV Hub config name \"statusLight\"");
        telemetry.update();

        ElapsedTime colorTimer = new ElapsedTime();
        int colorIndex = 0;

        // Cycle through colors continuously during INIT so the hardware mapping can be
        // verified before ever pressing START.
        while (!isStarted() && !isStopRequested()) {
            if (colorTimer.seconds() >= SECONDS_PER_COLOR) {
                colorIndex = (colorIndex + 1) % TEST_POSITIONS.length;
                colorTimer.reset();
            }

            double currentPosition = TEST_POSITIONS[colorIndex];
            robot.setStatusLightColor(currentPosition);

            telemetry.addData("Status", "Cycling colors during INIT - watch the light!");
            telemetry.addData("statusLight hardware mapped?",
                    robot.statusLight != null ? "YES" : "NO - check REV Hub config name \"statusLight\"");
            telemetry.addData("Commanded Position", "%.3f", currentPosition);
            telemetry.update();
        }

        waitForStart();

        // Keep cycling after START too, in case it's easier to observe the light with the
        // Driver Station no longer in the INIT screen.
        while (opModeIsActive()) {
            if (colorTimer.seconds() >= SECONDS_PER_COLOR) {
                colorIndex = (colorIndex + 1) % TEST_POSITIONS.length;
                colorTimer.reset();
            }

            double currentPosition = TEST_POSITIONS[colorIndex];
            robot.setStatusLightColor(currentPosition);

            telemetry.addData("Status", "RUNNING - cycling colors (press STOP to end test)");
            telemetry.addData("statusLight hardware mapped?",
                    robot.statusLight != null ? "YES" : "NO - check REV Hub config name \"statusLight\"");
            telemetry.addData("Commanded Position", "%.3f", currentPosition);
            telemetry.update();
        }
    }
}
