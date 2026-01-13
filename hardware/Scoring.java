
package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.stuyfission.fissionlib.input.GamepadStatic;
import com.stuyfission.fissionlib.util.Mechanism;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.drive.FieldCentricDrivetrain;
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
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    // ------------------ MODES / STATE ------------------
    private enum Mode {
        DRIVER,        // normal tele-op driving
        GO_TO_FAR_TIP, // go to far tip
        AUTO_ALIGN,    // using Limelight to center on tag
        GO_TO_CLOSE_TIP
    }

    private Mode mode = Mode.DRIVER;

    // ------------------ TUNABLE CONSTANTS (Dashboard) ------------------

    // Desired distance from tag when aligned (meters, robot-space Limelight measurement)
    public static double DESIRED_FWD_METERS = 1.5;

    // Gains for local auto-align movement (robot space)
    public static double FORWARD_GAIN = 1.0;
    public static double STRAFE_GAIN = 1.0;
    public static double TURN_GAIN = 0.03;

    // Max speed while auto-aligning / pathing (0..1 drive power)
    public static double MAX_AUTO_SPEED = 0.6;
    public static double MAX_AUTO_TURN_SPEED = .4;

    // Alignment tolerances (local/tag-relative)
    public static double X_TOLERANCE = 0.03;          // side error (m)
    public static double Y_TOLERANCE = 0.05;          // forward error (m)
    public static double YAW_TOLERANCE_DEG = 2.0;     // angle error (deg)


    // -------- Far shooting zone target pose (RoadRunner units: inches, radians) --------
    // Fill these from real field measurements (pose of the tip of the far shooting zone)
    public static double FAR_TIP_X_INCHES = 0.0;       // TODO: measure & set
    public static double FAR_TIP_Y_INCHES = -48;       // TODO: measure & set
    public static double FAR_TIP_HEADING_DEG = 0.0;    // TODO: measure & set
    public static double CLOSE_TIP_X_INCHES = 0.0;       // TODO: measure & set
    public static double CLOSE_TIP_Y_INCHES = 0.0;       // TODO: measure & set
    public static double CLOSE_TIP_HEADING_DEG = 0.0;    // TODO: measure & set

    // Tolerances for "we've reached the far tip pose"
    public static double FAR_POS_TOL_INCHES = 2.0;     // how close in XY before we switch to AUTO_ALIGN
    public static double FAR_HEADING_TOL_DEG = 3.0;    // heading tolerance at far tip

    // P gains for global approach (toward FAR_TIP pose)
    public static double FAR_KP_TRANSLATION = 0.03;    // drive power per inch of error
    public static double FAR_KP_ROTATION = 0.04;       // drive power per rad of error
    public static double CLOSE_POS_TOL_INCHES = 2.0;     // how close in XY before we switch to AUTO_ALIGN
    public static double CLOSE_HEADING_TOL_DEG = 3.0;    // heading tolerance at far tip

    // P gains for global approach (toward FAR_TIP pose)
    public static double CLOSE_KP_TRANSLATION = 0.03;    // drive power per inch of error
    public static double CLOSE_KP_ROTATION = 0.04;
    private boolean Red;
    private int id = 20;
    private boolean shoot = true;
    private double headingOffset;
    private Command transferCom = () -> transfer.run();
    private Command transferStop = () -> transfer.stop();
    private CommandSequence transSeq = new CommandSequence()
            .addWaitCommand(.3)
            .addCommand(transferCom)
            .addWaitCommand(.3)
            .addCommand(transferStop)
            .build();


    // ------------------ CONSTRUCTOR ------------------
    public Scoring(LinearOpMode opMode) {
        this.opMode = opMode;

        drivetrain = new Drivetrain(opMode);
        limelight = new Limelight(opMode);
        intake = new Intake(opMode);
        transfer = new Transfer(opMode);
        shooter = new Shooter(opMode);
    }
    public Shooter getShooter(){
        return this.shooter;
    }
    public Scoring(LinearOpMode opMode, boolean Red) {
        this.opMode = opMode;
        this.Red = Red;
        if (this.Red){
            id = 24;
        }
        else{
            id = 20;
        }
        drivetrain = new Drivetrain(opMode);
        limelight = new Limelight(opMode);
        intake = new Intake(opMode);
        transfer = new Transfer(opMode);
        shooter = new Shooter(opMode);
    }

    // ------------------ INIT ------------------
    @Override
    public void init(HardwareMap hwMap) {
        drivetrain.init(hwMap);
        limelight.init(hwMap);
        intake.init(hwMap);
        transfer.init(hwMap);
        shooter.init(hwMap);
    }

    public void init(HardwareMap hwMap, boolean Red) {
        drivetrain.init(hwMap);
        limelight.init(hwMap, Red);
        intake.init(hwMap);
        transfer.init(hwMap);
        shooter.init(hwMap);
        this.Red = Red;
        if (Red){
            drivetrain.setPoseEstimate(new Pose2d(24,-72,0));

        }
        else{
            drivetrain.setPoseEstimate(new Pose2d(-24,-72,0));
        }
        if (Red){
            id = 24;
        }
        else{
            id = 20;
        }
        drivetrain.update();
        headingOffset = drivetrain.getPoseEstimate().getHeading();
        shooter.shoot();
    }

    // ------------------ TELEMETRY ------------------
    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Mode", mode);

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

        double rpm1 = shooter.motors[0].getVelocity();
        double rpm2 = shooter.motors[1].getVelocity();
        double rpm = Math.max(rpm1,rpm2);
        double currentRpm = rpm * 60.0 / shooter.motors[0].getMotorType().getTicksPerRev();
        telemetry.addData("Shooter RPM", currentRpm);
        telemetry.addData("Ideal Close RPM", Shooter.closeShootRPM);
        telemetry.addData("Ideal Far RPM", Shooter.farShootRPM);

    }

    // ------------------ PUBLIC HELPERS ------------------

    /**
     * Start full sequence: drive from wherever you are to the far shooting zone tip,
     * then drop into tag-based AUTO_ALIGN (and eventually SHOOTING if enabled).
     */
    public void requestAutoAlignAndShoot() {
            mode = Mode.AUTO_ALIGN;
    }

    /**
     * Same idea; conceptually "go to center of shooting zone, aim, and fire".
     */
    public void requestCenterShot() {
        requestAutoAlignAndShoot();
    }


    private Pose2d getFarTipPose() {
        return new Pose2d(
                FAR_TIP_X_INCHES,
                FAR_TIP_Y_INCHES,
                Math.toRadians(FAR_TIP_HEADING_DEG)
        );
    }
    private Pose2d getCloseTipPose() {
        return new Pose2d(
                CLOSE_TIP_X_INCHES,
                CLOSE_TIP_Y_INCHES,
                Math.toRadians(CLOSE_TIP_HEADING_DEG)
        );
    }

    // ------------------ MAIN LOOP ------------------
    @Override
    public void loop(Gamepad gamepad) {
        drivetrain.update();
        // Use RR heading (radians) as orientation prior for MegaTag2
        Pose2d rrPose = drivetrain.getPoseEstimate();
        double headingDeg = Math.toDegrees(rrPose.getHeading());

        // Update Limelight; this fills tag-relative data and global botpose_MT2
        limelight.update(headingDeg);

        // OPTIONAL: if you want to fuse vision pose into RR, uncomment:
        if (mode != Mode.DRIVER) {
            Pose2d visionPose = limelight.getGlobalPose();
            if (visionPose != null) {
                Pose2d rr = drivetrain.getPoseEstimate();
                drivetrain.setPoseEstimate(new Pose2d(visionPose.getX(), visionPose.getY(), rr.getHeading()));
            }
        }


        // Always handle intake / outtake
        handleIntake(gamepad);


        // Handle scoring button and abort button
        handleButtons(gamepad);

        // Run control logic based on current mode
        switch (mode) {
            case DRIVER:
                limelight.stop();
                manualDrive(gamepad);
                break;
           case GO_TO_FAR_TIP:
               limelight.start();
               if (Math.abs(gamepad.left_stick_x) > 0.05 ||
                       Math.abs(gamepad.left_stick_y) > 0.05 ||
                       Math.abs(gamepad.right_stick_x) > 0.05) {
                   mode = Mode.DRIVER;
                   manualDrive(gamepad);
               } else {
                   goToFarTipStep();
               }
               break;
            case GO_TO_CLOSE_TIP:
                limelight.start();
                if (Math.abs(gamepad.left_stick_x) > 0.05 ||
                        Math.abs(gamepad.left_stick_y) > 0.05 ||
                        Math.abs(gamepad.right_stick_x) > 0.05) {
                    mode = Mode.DRIVER;
                    manualDrive(gamepad);
                } else {
                    goToCloseTipStep();
                }
                break;
            case AUTO_ALIGN:
                limelight.start();
                if (Math.abs(gamepad.left_stick_x) > 0.05 ||
                        Math.abs(gamepad.left_stick_y) > 0.05 ||
                        Math.abs(gamepad.right_stick_x) > 0.05) {
                    mode = Mode.DRIVER;
                    manualDrive(gamepad);
                } else {
                    autoAlignStep();
                }
                break;
        }
    }
    // ------------------ INPUT HANDLING ------------------
    private void handleButtons(Gamepad gamepad) {
        if (GamepadStatic.isButtonPressed(gamepad, Controls.GOTOP)) {
            mode = Mode.GO_TO_CLOSE_TIP;
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.GOBOTTOM)){
            mode = Mode.GO_TO_FAR_TIP;
        }
        else if(GamepadStatic.isButtonPressed(gamepad,Controls.ALIGN)){
            mode = Mode.AUTO_ALIGN;
        }
        else{
            mode = Mode.DRIVER;
        }

        if (GamepadStatic.isButtonPressed(gamepad,Controls.RESETHEADING)){
            drivetrain.resetIMU();
            drivetrain.update();
            headingOffset = drivetrain.getPoseEstimate().getHeading();
        }

        if (GamepadStatic.isButtonPressed(gamepad, Controls.FARSHOOT)){
            shooter.setFar(true);
            shoot = true;
            transSeq.trigger();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.CLOSESHOOT)){
            shooter.setFar(false);
            shoot = true;
            transSeq.trigger();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.UNSHOOT)){
            shooter.unshoot();
            transfer.backup();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.STOP)){
            shoot = false;
        }
        else {
            if (shoot) {
                shooter.shoot();
            }
            else{
                shooter.stop();
            }
        }
    }

    private void handleIntake(Gamepad gamepad){
        // INTAKE: A
        // OUTTAKE: left bumper
        if (GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)) {
            intake.intake();
            transfer.intake();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.OUTTAKE)) {
            //intake.setOut(intake.getOut() + .1);
            intake.outtake();
            transfer.backup();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.TRANSFER)){
            transfer.run();
        }
        else if (GamepadStatic.isButtonPressed(gamepad, Controls.UNTRANSFER)){
            transfer.backup();
        }
        else{
            intake.resetOut();
            intake.stop();
            if (!(GamepadStatic.isButtonPressed(gamepad, Controls.INTAKE)||
                    GamepadStatic.isButtonPressed(gamepad,Controls.TRANSFER)||
                    GamepadStatic.isButtonPressed(gamepad,Controls.OUTTAKE)||
                    GamepadStatic.isButtonPressed(gamepad,Controls.UNTRANSFER)||
                    GamepadStatic.isButtonPressed(gamepad,Controls.FARSHOOT)||
                    GamepadStatic.isButtonPressed(gamepad,Controls.CLOSESHOOT)||
                    GamepadStatic.isButtonPressed(gamepad,Controls.UNSHOOT))){
                transfer.stop();
            }
        }
    }

    // ------------------ DRIVING ------------------

    private void manualDrive(Gamepad gamepad) {
        drivetrain.update();
        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        gamepad.left_stick_y,
                        -gamepad.left_stick_x * 1.1,
                        gamepad.right_stick_x ));

        drivetrain.update();

    }
    /**
     * Global approach to reach the far shooting zone tip using RoadRunner pose.
     * Once we are close enough to FAR_TIP pose, we switch to AUTO_ALIGN for fine tag-based alignment.
     */
    private void goToFarTipStep() {
        shooter.setFar(true);
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
        turnCmd = clamp(turnCmd, -MAX_AUTO_TURN_SPEED, MAX_AUTO_TURN_SPEED);


        drivetrain.setDrivePower(new Pose2d(forwardCmd, strafeCmd, turnCmd));
    }
    private void goToCloseTipStep() {
        shooter.setFar(false);
        Pose2d pose = drivetrain.getPoseEstimate();
        Pose2d target = getCloseTipPose();

        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        double distance = Math.hypot(dx, dy);

        double targetHeading = target.getHeading();
        double headingError = angleWrap(targetHeading - pose.getHeading());

        boolean atPosition = distance < CLOSE_POS_TOL_INCHES;
        boolean atHeading = Math.abs(Math.toDegrees(headingError)) < CLOSE_HEADING_TOL_DEG;

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

        double forwardCmd = CLOSE_KP_TRANSLATION * robotXError;
        double strafeCmd  = CLOSE_KP_TRANSLATION * robotYError;
        double turnCmd    = CLOSE_KP_ROTATION * headingError;

        forwardCmd = clamp(forwardCmd, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafeCmd  = clamp(strafeCmd,  -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turnCmd = clamp(turnCmd, -MAX_AUTO_TURN_SPEED, MAX_AUTO_TURN_SPEED);


        drivetrain.setDrivePower(new Pose2d(forwardCmd, strafeCmd, turnCmd));
    }

    /**
     * Local tag-relative alignment using Limelight helper.
     * Runs after we’re already at the far tip pose.
     */
    private void autoAlignStep() {
            Limelight.Location loc = limelight.getBest();

            // If no tag or wrong tag ID: spin to search
            if (loc.tagID < 0 || loc.tagID != id) {
                drivetrain.setDrivePower(new Pose2d(0, 0,
                        Red ? -MAX_AUTO_TURN_SPEED : MAX_AUTO_TURN_SPEED));
                return;
            }

            double fwdError = loc.y - DESIRED_FWD_METERS;

            boolean aligned =
                    Math.abs(loc.x)    < X_TOLERANCE &&
                            Math.abs(fwdError) < Y_TOLERANCE &&
                            Math.abs(loc.yaw)  < YAW_TOLERANCE_DEG;

            // Compute desired robot-centric drive commands from Limelight
            Limelight.DriveCommands cmds = limelight.computeDriveCommands(
                    DESIRED_FWD_METERS, FORWARD_GAIN, STRAFE_GAIN, TURN_GAIN
            );

            if (!aligned) {
                // Still aligning → drive toward target
                drivetrain.setDrivePower(new Pose2d(
                        clamp(cmds.forward, -MAX_AUTO_SPEED,       MAX_AUTO_SPEED),
                        clamp(cmds.strafe,  -MAX_AUTO_SPEED,       MAX_AUTO_SPEED),
                        clamp(cmds.turn,    -MAX_AUTO_TURN_SPEED,  MAX_AUTO_TURN_SPEED)
                ));
            } else {
                // Aligned → stop driving and run transfer/shoot
                drivetrain.setDrivePower(new Pose2d(0, 0, 0));
                transfer.run();   // or start your shooting state
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
