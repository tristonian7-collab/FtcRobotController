## TeamCode Module

This module holds our team's robot controller code: a 4-motor Mecanum drivetrain, a closed-loop
shooter flywheel, an intake/feeder group, an AprilTag webcam, and a status light. All OpModes
share one hardware class so device names, directions and setup stay identical everywhere.

### Shared hardware: `RobotHardware`

Create one instance per OpMode and call `robot.init(hardwareMap)` first. Config names:

| Device | Config name | Required? |
|---|---|---|
| Drive motors | `frontLeftDrive`, `frontRightDrive`, `backLeftDrive`, `backRightDrive` | Yes |
| Shooter flywheel | `shooterMotor` | Yes |
| Intake motor | `feederMotor` | Optional |
| Feeder gate servo (positional) | `feederServo` | Optional |
| Pollen feeder servo (positional) | `pollenFeederServo` | Optional |
| Status light (goBilda, servo port 1) | `statusLight` | Optional |
| IMU | `imu` | Optional |
| Webcam | `Webcam 1` (mapped by each vision OpMode, not `RobotHardware`) | Optional |

Optional devices are left `null` when missing, so null-check them. `leftServo`/`rightServo` were
removed because they are no longer on the robot.

Key helpers: `driveMecanum(forward, strafe, turn)` (positive strafe = right, negative = left),
`stopDrive()`, `setIntakePower()`, `setFeederServoPosition()`, `setPollenFeederServoPosition()`,
`setTargetShooterRPM()` / `getMotorRPM()`, and `setStatusLightWhite/Green/Red()`.

Shooter speed is closed-loop (RPM via `DcMotorEx` velocity). The only shooter API is RPM; the old
power-based API is gone. The shared default is `RobotHardware.DEFAULT_SHOOTER_RPM` (25.0), used by
every OpMode.

Servo angles assume a 300 degree full range (`*_FULL_RANGE_DEGREES`), which is still an
uncalibrated TODO. If a servo moves the wrong way, flip its direction in `RobotHardware.init`.

`hardware/TestHardware.java` is a parallel copy of the hardware class for experiments.

### OpModes

| OpMode (Driver Station name) | Class | Type | What it does |
|---|---|---|---|
| MecanumStraferChassis | `MecanumStraferChassis` | TeleOp | Main driving OpMode (controls below). |
| primaryshooterauto | `PrimaryShooterAuto` | Auto | Feeder/gate shooting sequence; wheels currently disabled for testing. |
| secondaryparking | `SecondaryParking` | Auto | Spins up the shooter and watches for an AprilTag for up to 15 s. If one is seen it fires (about 3 s), then always parks: 60 cm backward, then 180 cm left. |
| AprilTag Auto Firing | `AprilTagAutonomous` | Auto | Tracks an AprilTag and fires. |
| AprilTag Light Detector | `AprilTagLightDetector` | TeleOp | Vision diagnostic. |
| AprilTag Distance Test | `DistanceTest` | TeleOp | Reads tag range for calibration. |
| Status Light Test | `StatusLightTest` | TeleOp | Cycles the status light to verify wiring and config. |
| EncoderTest | `EncoderTest` | TeleOp | Runs the shooter at the default RPM and shows revs and RPM. |

`CornerAuto` ("secondaryparking2", a 60 cm strafe-only auto) was removed.

### TeleOp controls (gamepad 1, `MecanumStraferChassis`)

* Left stick: drive and strafe. Right stick X: turn.
* Right trigger: intake motor power.
* Triangle (hold): feeder gate up (120 degrees); release to return to the bottom.
* Left trigger: manual override that sets the feeder gate position directly. It takes priority over Triangle.
* Square (hold): pollen feeder servo moves from its 120 degree rest to 20 degrees.
* The shooter runs continuously at the default RPM.
* Status light: white = no tag, red = tag seen but out of range, green = tag in range (130-150 cm).
  The light cycles through colors during INIT to confirm wiring.

### Autonomous conventions

* Movement is time-based: distance divided by `CM_PER_SECOND` (71.33) at `DRIVE_SPEED` 0.5, with 500 ms ramp-up and ramp-down. There is no encoder feedback.
* AprilTag range is corrected with `DISTANCE_CALIBRATION_SCALE` = (160/145) * 1.02.
* Vision setup is wrapped in try/catch, so OpModes still run if the webcam is missing.

### Git

Branch is `master`; remote is `tristonian7-collab/FtcRobotController`. On Windows, git may not be on
the PATH inside Android Studio's terminal. Use `C:\Program Files\Git\cmd\git.exe`. Pushes
authenticate through Git Credential Manager (the "Connect to GitHub" window).

---

## Adding OpModes (general FTC guidance)

The process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

### Naming of Samples

To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""