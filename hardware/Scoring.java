
package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

/**
 * High-level scoring controller:
 * - Manual drive + intake/outtake
 * - One-button: go to far shooting zone tip, then auto-align to AprilTag
 * - Optional shooting sequence (currently commented out)
 */
@Config
public class Scoring extends Mechanism {

    // ------------------ SUBSYSTEMS ------------------
    private Drivetrain drivetrain;
    private Limelight limelight;
    //private final Intake intake;
    //private final Transfer transfer;
    //private final Shooter shooter;

    // ------------------ MODES / STATE ------------------
    private enum Mode {
        DRIVER,        // normal tele-op driving
        GO_TO_FAR_TIP, // drive (global) to far shooting zone tip
        AUTO_ALIGN,    // using Limelight to center on tag
        SHOOTING       // shooter spinup + feeding
    }

    private enum ShootStage {
        NONE,
        SPINUP,
        FEEDING
    }

    private Mode mode = Mode.DRIVER;
    private ShootStage shootStage = ShootStage.NONE;

    // timing for shooting
    private double stageStartTime = 0.0;

    // ------------------ TUNABLE CONSTANTS (Dashboard) ------------------

    // Desired distance from tag when aligned (meters, robot-space Limelight measurement)
    public static double DESIRED_FWD_METERS = 1.5;

    // Gains for local auto-align movement (robot space)
    public static double FORWARD_GAIN = 1.0;
    public static double STRAFE_GAIN = 1.0;
    public static double TURN_GAIN = 0.03;

    // Max speed while auto-aligning / pathing (0..1 drive power)
    public static double MAX_AUTO_SPEED = 0.6;
    public static double MAX_AUTO_TURN_SPEED = .5;

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
    public static double FAR_KP_ROTATION = 0.04;       // drive power per rad of error

    // ------------------ BUTTON EDGE FLAGS ------------------
    private boolean shootButtonLatched = false;
    private boolean abortButtonLatched = false;

    // ------------------ CONSTRUCTOR ------------------
    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;

        drivetrain = new Drivetrain(opMode);
        limelight = new Limelight(opMode);
        //intake = new Intake(opMode);
        //transfer = new Transfer(opMode);
        //shooter = new Shooter(opMode);
    }

    // ------------------ INIT ------------------
    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        limelight.init(hwMap);
        //intake.init(hwMap);
        //transfer.init(hwMap);
        //shooter.init(hwMap);
    }

    // ------------------ TELEMETRY ------------------
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Mode", mode);
        telemetry.addData("Shoot Stage", shootStage);

        // Show basic Limelight info (last best target)
        Limelight.Location loc = limelight.getBest();
        telemetry.addData("Tag ID", loc.tagID);
        telemetry.addData("LL x (m)", loc.x);
        telemetry.addData("LL y (m)", loc.y);
        telemetry.addData("LL yaw (deg)", loc.yaw);

        Pose2d pose = drivetrain.getPoseEstimate();
        telemetry.addData("RR Pose X (in)", pose.getX());
        telemetry.addData("RR Pose Y (in)", pose.getY());
        telemetry.addData("RR Pose H (deg)", Math.toDegrees(pose.getHeading()));
    }

    // ------------------ PUBLIC HELPERS ------------------

    /**
     * Start full sequence: drive from wherever you are to the far shooting zone tip,
     * then drop into tag-based AUTO_ALIGN (and eventually SHOOTING if enabled).
     */
    public void requestAutoAlignAndShoot() {
        if (mode == Mode.DRIVER) {
            mode = Mode.AUTO_ALIGN;
            shootStage = ShootStage.NONE;
        }
    }

    /**
     * Same idea; conceptually "go to center of shooting zone, aim, and fire".
     */
    public void requestCenterShot() {
        requestAutoAlignAndShoot();
    }

    /**
     * Abort any auto-align / shooting and return to driver control.
     */
    private void abortAuto() {
        mode = Mode.DRIVER;
        shootStage = ShootStage.NONE;
        drivetrain.setDrivePower(new Pose2d(0, 0, 0));
        //shooter.passivePower();
        //transfer.downPos();
    }

    private Pose2d getFarTipPose() {
        return new Pose2d(
                FAR_TIP_X_INCHES,
                FAR_TIP_Y_INCHES,
                Math.toRadians(FAR_TIP_HEADING_DEG)
        );
    }

    // ------------------ MAIN LOOP ------------------
    @Override
    public void loop(Gamepad gamepad) {
        // Update odometry & trajectory follower
        drivetrain.update();

        // Use RR heading (radians) as orientation prior for MegaTag2
        Pose2d rrPose = drivetrain.getPoseEstimate();
        double headingDeg = Math.toDegrees(rrPose.getHeading());

        // Update Limelight; this fills tag-relative data and global botpose_MT2
        limelight.update(headingDeg);

        // OPTIONAL: if you want to fuse vision pose into RR, uncomment:

        Pose2d visionPose = limelight.getGlobalPose();
        if (visionPose != null) {
            drivetrain.setPoseEstimate(visionPose);
        }


        // Always handle intake / outtake
        handleIntake(gamepad);


        // Handle scoring button and abort button
        handleButtons(gamepad);

        // Run control logic based on current mode
        switch (mode) {
            case DRIVER:
                manualDrive(gamepad);
                break;

           // case GO_TO_FAR_TIP:
              //  goToFarTipStep();
               // break;

            case AUTO_ALIGN:
                autoAlignStep();
                break;

            case SHOOTING:
                autoShootStep();
                break;
        }
    }

    // ------------------ INPUT HANDLING ------------------

    private void handleButtons(Gamepad gamepad) {
        boolean align = gamepad.right_bumper;
        if (GamepadStatic.isButtonPressed(gamepad, Controls.ALIGN) && !shootButtonLatched) {
            shootButtonLatched = true;
            requestAutoAlignAndShoot();
        } else if (!align) {
            shootButtonLatched = false;
        }

        boolean abortPressed = gamepad.dpad_down;
        if (GamepadStatic.isButtonPressed(gamepad, Controls.ABORT) && !abortButtonLatched) {
            abortButtonLatched = true;
            abortAuto();
        } else if (!abortPressed) {
            abortButtonLatched = false;
        }

        if (GamepadStatic.isButtonPressed(gamepad, Controls.SHOOT)){

            beginShooting();
        }
    }

    private void handleIntake(Gamepad gamepad) {
        // INTAKE: A
        // OUTTAKE: left bumper
        if (gamepad.a) {
            //intake.intake();
            System.out.println("intake");
        } else if (gamepad.left_bumper) {
            System.out.println("outtake");
        } else {
            System.out.println("stop");
        }
    }

    // ------------------ DRIVING ------------------

    private void manualDrive(Gamepad gamepad) {
        drivetrain.setDrivePower(new Pose2d(
                -gamepad.left_stick_y,   // forward/back
                -gamepad.left_stick_x,   // strafe
                -gamepad.right_stick_x   // rotate
        ));
    }

    /**
     * Global approach to reach the far shooting zone tip using RoadRunner pose.
     * Once we are close enough to FAR_TIP pose, we switch to AUTO_ALIGN for fine tag-based alignment.
     */
    private void goToFarTipStep() {
        Pose2d pose = drivetrain.getPoseEstimate();
        Pose2d target = getFarTipPose();

        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        double distance = Math.hypot(dx, dy);

        double targetHeading = target.getHeading();
        double headingError = angleWrap(targetHeading - pose.getHeading());

        boolean atPosition = distance < FAR_POS_TOL_INCHES;
        boolean atHeading = Math.abs(Math.toDegrees(headingError)) < FAR_HEADING_TOL_DEG;

        if (atPosition && atHeading) {
            drivetrain.setDrivePower(new Pose2d(0, 0, 0));
            // Hand off to Limelight-based fine alignment
            mode = Mode.AUTO_ALIGN;
            return;
        }

        // Convert field-space error (dx, dy) into robot-centric (forward, strafe)
        double heading = pose.getHeading();
        double cosH = Math.cos(-heading);
        double sinH = Math.sin(-heading);

        double robotXError = dx * cosH - dy * sinH; // forward
        double robotYError = dx * sinH + dy * cosH; // strafe

        double forwardCmd = FAR_KP_TRANSLATION * robotXError;
        double strafeCmd  = FAR_KP_TRANSLATION * robotYError;
        double turnCmd    = FAR_KP_ROTATION * headingError;

        forwardCmd = clamp(forwardCmd, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafeCmd  = clamp(strafeCmd,  -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turnCmd    = clamp(turnCmd,    -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

        drivetrain.setDrivePower(new Pose2d(forwardCmd, strafeCmd, turnCmd));
    }

    /**
     * Local tag-relative alignment using Limelight helper.
     * Runs after we’re already at the far tip pose.
     */
    private void autoAlignStep() {
        Limelight.Location loc = limelight.getBest();

        // If no tag seen, slowly spin in place to search
        if (loc.tagID < 0) {
            drivetrain.setDrivePower(new Pose2d(0, 0, +MAX_AUTO_TURN_SPEED));
            return;
        }
        else{
            drivetrain.setDrivePower(new Pose2d(0, 0, 0));
        }

        // Compute drive commands from Limelight helper (distance-aware gains)
        Limelight.DriveCommands cmds = limelight.computeDriveCommands(
                DESIRED_FWD_METERS, FORWARD_GAIN, STRAFE_GAIN, TURN_GAIN
        );
        // Check alignment tolerances
        double fwdError = loc.y - DESIRED_FWD_METERS;

        boolean aligned =
                Math.abs(loc.x)      < X_TOLERANCE &&
                        Math.abs(fwdError)   < Y_TOLERANCE &&
                        Math.abs(loc.yaw)    < YAW_TOLERANCE_DEG;

        if (aligned) {
            // Stop and begin shooting sequence (or just drop back to DRIVER if you don't want auto-shoot yet)
            drivetrain.setDrivePower(new Pose2d(0, 0, 0));
            //beginShooting();
            // If you don't want auto-shoot at all yet, comment the above line and just do:
            mode = Mode.DRIVER;
        }
    }

    private void beginShooting() {
        mode = Mode.SHOOTING;
        shootStage = ShootStage.SPINUP;
        stageStartTime = opMode.getRuntime();

        //shooter.shoot();      // spin shooter up
        //transfer.downPos();   // make sure transfer is down while spinning up
    }

    private void autoShootStep() {
        double now = opMode.getRuntime();
        drivetrain.setDrivePower(new Pose2d(0, 0, 0)); // stay still while shooting

        switch (shootStage) {
            case SPINUP:
                if (now - stageStartTime >= SHOOT_SPINUP_TIME) {
                    // Start feeding balls
                    //transfer.upPos();
                    shootStage = ShootStage.FEEDING;
                    stageStartTime = now;
                }
                break;

            case FEEDING:
                if (now - stageStartTime >= SHOOT_FEED_TIME) {
                    // Done shooting, reset
                    //transfer.downPos();
                    //shooter.passivePower();
                    shootStage = ShootStage.NONE;
                    mode = Mode.DRIVER;
                }
                break;

            case NONE:
            default:
                // Safety fallback
                mode = Mode.DRIVER;
                break;
        }
    }

    // ------------------ UTIL ------------------
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
