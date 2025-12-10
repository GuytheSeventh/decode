
package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Limelight.Location;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.autoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class auto extends LinearOpMode {

    public auto(boolean Red) {
        this.Red = Red;
    }

    private boolean Red;
    private boolean commandBusy = false;
    private Pose2d targetPoint = null;
    private Pose2d drivePos = null;
    private Location loc = null;
    private TrajectorySequence preloadTraj;
    private TrajectorySequence back2wallTraj;
    private TrajectorySequence farSampleIntTraj;
    private TrajectorySequence basket2Traj;
    private TrajectorySequence centerSampleTraj;
    private TrajectorySequence centerSampleIntTraj;
    private TrajectorySequence basket3Traj;
    private TrajectorySequence wallSampleTraj;
    private TrajectorySequence wallSampleIntTraj;
    private TrajectorySequence basket4Traj;
    private TrajectorySequence sub1Traj;
    private TrajectorySequence basket5Traj;
    private TrajectorySequence sub2Traj;
    private TrajectorySequence basket6Traj;
    private TrajectorySequence sub3Traj;
    private TrajectorySequence basket7Traj;
    private TrajectorySequence sub4Traj;
    private TrajectorySequence basket8Traj;
    private TrajectorySequence sub5Traj;
    private TrajectorySequence basket9Traj;

    private SampleMecanumDrive drive;

    private Command preloadCommand = () -> drive.followTrajectorySequenceAsync(preloadTraj);
    private Command back2wallCommand = () -> drive.followTrajectorySequenceAsync(back2wallTraj);
    private Command farSampleIntCommand = () -> drive.followTrajectorySequenceAsync(farSampleIntTraj);
    private Command basket2Command = () -> drive.followTrajectorySequenceAsync(basket2Traj);
    private Command centerSampleCommand = () -> drive.followTrajectorySequenceAsync(centerSampleTraj);
    private Command centerSampleIntCommand = () -> drive.followTrajectorySequenceAsync(centerSampleIntTraj);
    private Command basket3Command = () -> drive.followTrajectorySequenceAsync(basket3Traj);
    private Command wallSampleCommand = () -> drive.followTrajectorySequenceAsync(wallSampleTraj);
    private Command wallSampleIntCommand = () -> drive.followTrajectorySequenceAsync(wallSampleIntTraj);
    private Command basket4Command = () -> drive.followTrajectorySequenceAsync(basket4Traj);
    private Command sub1Command = () -> drive.followTrajectorySequenceAsync(sub1Traj);
    private Command basket5Command = () -> drive.followTrajectorySequenceAsync(basket5Traj);
    private Command sub2Command = () -> drive.followTrajectorySequenceAsync(sub2Traj);
    private Command basket6Command = () -> drive.followTrajectorySequenceAsync(basket6Traj);
    private Command sub3Command = () -> drive.followTrajectorySequenceAsync(sub3Traj);
    private Command basket7Command = () -> drive.followTrajectorySequenceAsync(basket7Traj);
    private Command sub4Command = () -> drive.followTrajectorySequenceAsync(sub4Traj);
    private Command basket8Command = () -> drive.followTrajectorySequenceAsync(basket8Traj);
    private Command sub5Command = () -> drive.followTrajectorySequenceAsync(sub5Traj);
    private Command basket9Command = () -> drive.followTrajectorySequenceAsync(basket9Traj);

    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Limelight limelight;

    private Command commandBusyTrue = () -> commandBusy = true;
    private Command commandBusyFalse = () -> commandBusy = false;
    private Command outtake = () -> intake.outtake();
    private Command stopIntake = () -> intake.stop();
    private Command intakeCommand = () -> intake.intake();
    private Command transferUp = () -> transfer.run();
    private Command transferDown = () -> transfer.backup();
    private Command transferStop = () -> transfer.stop();
    private Command shoot = () -> shooter.shoot();
    private Command passive = () -> shooter.passivePower();
    public double shootHood = .5;
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX() + 2,
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
        targetPoint = null;
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };

    private CommandSequence preload = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(preloadCommand)
            .addWaitCommand(1)
            .addCommand(back2wallCommand)
            .addCommand(commandBusyFalse)
            .build();

    private CommandSequence doNothing = new CommandSequence().build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(preload)
            .addCommandSequence(doNothing)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        VoltageSensor voltage = hardwareMap.voltageSensor.iterator().next();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(this);
        drive = new SampleMecanumDrive(hardwareMap);
        transfer = new Transfer(this);
        shooter = new Shooter(this);
        limelight = new Limelight(this);

        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        shooter.init(hardwareMap);
        limelight.init(hardwareMap, Red);

        TrajectoryVelocityConstraint fastDT = SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint fastDTA = SampleMecanumDrive.getAccelerationConstraint(70);

        preloadTraj = drive
                .trajectorySequenceBuilder(autoConstants.START.getPose())
                .splineToLinearHeading(autoConstants.farTip.getPose(), 5 * Math.PI / 4)
                .build();
        telemetry.addLine("Built preloadTraj");
        telemetry.update();
        back2wallTraj = drive
                .trajectorySequenceBuilder(preloadTraj.end())
                .setReversed(false)
                .splineToLinearHeading(autoConstants.back2wall.getPose(), Math.PI / 2)
                .build();
        telemetry.addLine("Built back2wallTraj");
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            drive.updatePoseEstimate();
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.addData("voltage", voltage.getVoltage());
            telemetry.update();
        }

        drive.setPoseEstimate(autoConstants.START.getPose());

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            if (targetPoint != null) {
                Drive.p2p(drive, targetPoint, voltage.getVoltage());
            }
            drive.update();
            limelight.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.update();
        }

        limelight.stop();
        Thread.sleep(500);
    }
}
