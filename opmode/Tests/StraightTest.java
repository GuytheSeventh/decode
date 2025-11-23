package org.firstinspires.ftc.teamcode.opmode.Tests;

import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {

    private Robot robot = new Robot(this);

    private Limelight limelight = new Limelight(this);

    public static double DESIRED_FWD_METERS = 1.5;
    public static double FORWARD_GAIN = 1.0;
    public static double STRAFE_GAIN = 1.0;
    public static double TURN_GAIN = 0.03;

    // Max speed while auto-aligning / pathing (0..1 drive power)
    public static double MAX_AUTO_SPEED = 0.6;

    // Alignment tolerances (local/tag-relative)
    public static double X_TOLERANCE = 0.03;          // side error (m)
    public static double Y_TOLERANCE = 0.05;          // forward error (m)
    public static double YAW_TOLERANCE_DEG = 2.0;     // angle error (deg)

    // Shooter timing
    public static double SHOOT_SPINUP_TIME = 0.5;     // seconds to spin up shooter
    public static double SHOOT_FEED_TIME = 1.0;       // seconds to feed balls

    // -------- Far shooting zone target pose (RoadRunner units: inches, radians) --------
    // Fill these from real field measurements (pose of the tip of the far shooting zone)
    public static double FAR_TIP_X_INCHES = 0.0;       // TODO: measure & set
    public static double FAR_TIP_Y_INCHES = 0.0;       // TODO: measure & set
    public static double FAR_TIP_HEADING_DEG = 0.0;    // TODO: measure & set

    // Tolerances for "we've reached the far tip pose"
    public static double FAR_POS_TOL_INCHES = 2.0;     // how close in XY before we switch to AUTO_ALIGN
    public static double FAR_HEADING_TOL_DEG = 3.0;    // heading tolerance at far tip

    // P gains for global approach (toward FAR_TIP pose)
    public static double FAR_KP_TRANSLATION = 0.03;    // drive power per inch of error
    public static double FAR_KP_ROTATION = 0.04;

    private Drivetrain drivetrain = new Drivetrain(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Limelight.Location loc = limelight.getBest();

            // If no tag seen, slowly spin in place to search
            if (loc.tagID < 0) {
                drivetrain.setDrivePower(new Pose2d(0, 0, -0.06));
                return;
            }

            // Compute drive commands from Limelight helper (distance-aware gains)
            Limelight.DriveCommands cmds = limelight.computeDriveCommands(
                    DESIRED_FWD_METERS, FORWARD_GAIN, STRAFE_GAIN, TURN_GAIN
            );

            double forward = clamp(cmds.forward, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double strafe  = clamp(cmds.strafe,  -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn    = clamp(cmds.turn,    -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

            drivetrain.setDrivePower(new Pose2d(forward, strafe, turn));

            // Check alignment tolerances
            double fwdError = loc.y - DESIRED_FWD_METERS;

            boolean aligned =
                    Math.abs(loc.x)      < X_TOLERANCE &&
                            Math.abs(fwdError)   < Y_TOLERANCE &&
                            Math.abs(loc.yaw)    < YAW_TOLERANCE_DEG;

            if (aligned) {
                // Stop and begin shooting sequence (or just drop back to DRIVER if you don't want auto-shoot yet)
                drivetrain.setDrivePower(new Pose2d(0, 0, 0));
                // If you don't want auto-shoot at all yet, comment the above line and just do:
                // mode = Mode.DRIVER;
            }
        }
    }
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // Wrap angle to [-pi, pi]
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}

