
package org.firstinspires.ftc.teamcode.opmode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.command.AutoCommandMachine;
import com.stuyfission.fissionlib.command.Command;
import com.stuyfission.fissionlib.command.CommandSequence;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Limelight.Location;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.opmode.auton.autoConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

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
    private TrajectorySequence closeBallTraj;
    private TrajectorySequence shoot1Traj;
    private TrajectorySequence midBallTraj;
    private TrajectorySequence shoot2Traj;
    private TrajectorySequence farBallTraj;
    private TrajectorySequence shoot3Traj;
    private TrajectorySequence goHomeTraj;

    private SampleMecanumDrive drive;

    private Command preloadCommand = () -> drive.followTrajectorySequenceAsync(preloadTraj);
    private Command goHomeCommand = () -> drive.followTrajectorySequenceAsync(goHomeTraj);
    private Command closeBallCommand = () -> drive.followTrajectorySequenceAsync(closeBallTraj);
    private Command shoot1Command = () -> drive.followTrajectorySequenceAsync(shoot1Traj);
    private Command midBallCommand = () -> drive.followTrajectorySequenceAsync(midBallTraj);
    private Command shoot2Command = () -> drive.followTrajectorySequenceAsync(shoot2Traj);
    private Command farBallCommand = () -> drive.followTrajectorySequenceAsync(farBallTraj);
    private Command shoot3Command = () -> drive.followTrajectorySequenceAsync(shoot3Traj);

    private Command forward = () -> drive.followTrajectoryAsync(
            drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(10)
                    .build()
    );
    private Command rotate = () -> drive.followTrajectoryAsync(
            drive.trajectoryBuilder(drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(0,0),0)
                    .build()
    );

    private Command backward = () -> drive.followTrajectoryAsync(
            drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(10)
                    .build()
    );


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
    private Command transferIntake = () -> transfer.intake();
    private Command transferStop = () -> transfer.stop();
    private Command shoot = () -> shooter.shoot();
    private Command shootStop = () -> shooter.stop();
    private Command passive = () -> shooter.passivePower();
    private Command forwardP2P = () -> targetPoint = new Pose2d(targetPoint.getX(),
            targetPoint.getY(), targetPoint.getHeading());
    private Command driveStop = () -> {
        targetPoint = null;
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    };
    private CommandSequence preload = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(preloadCommand)
            //.addCommand(transferUp)
            .addWaitCommand(1.5)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence closeBall = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(closeBallCommand)
           // .addCommand(intakeCommand)
           // .addCommand(transferIntake)
            .addWaitCommand(.5)
            //.addCommand(forward)
           // .addWaitCommand(.1)
           // .addCommand(backward)
          //  .addCommand(stopIntake)
           // .addCommand(transferStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence shoot1 = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(shoot1Command)
            //.addCommand(transferUp)
            .addWaitCommand(1.5)
            //.addCommand(transferStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence midBall = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(midBallCommand)
            .addCommand(intakeCommand)
            .addCommand(transferIntake)
            .addWaitCommand(.5)
            //.addCommand(forward)
            // .addWaitCommand(.1)
            // .addCommand(backward)
            .addCommand(stopIntake)
            .addCommand(transferStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence shoot2 = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(shoot2Command)
            .addCommand(transferUp)
            .addWaitCommand(1.5)
            .addCommand(transferStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence farBall = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(farBallCommand)
            .addCommand(intakeCommand)
            .addCommand(transferIntake)
            .addWaitCommand(.5)
            //.addCommand(forward)
            // .addWaitCommand(.1)
            // .addCommand(backward)
            .addCommand(stopIntake)
            .addCommand(transferStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence shoot3 = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(shoot3Command)
            .addCommand(transferUp)
            .addWaitCommand(1.5)
            .addCommand(transferStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence goHome = new CommandSequence()
            .addCommand(commandBusyTrue)
            .addCommand(goHomeCommand)
            .addWaitCommand(1)
            .addCommand(shootStop)
            .addCommand(commandBusyFalse)
            .build();
    private CommandSequence doNothing = new CommandSequence().build();
    private AutoCommandMachine commandMachine = new AutoCommandMachine()
            .addCommandSequence(preload)
            //.addCommandSequence(closeBall)
            //.addCommandSequence(shoot1)
            //.addCommandSequence(midBall)
            //.addCommandSequence(shoot2)
            //.addCommandSequence(farBall)
            //.addCommandSequence(shoot3)
            .addCommandSequence(goHome)
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




        TrajectoryVelocityConstraint fastDT = SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL,
                DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint fastDTA = SampleMecanumDrive.getAccelerationConstraint(25);

        preloadTraj = drive
                .trajectorySequenceBuilder(autoConstants.START.getPose())
                .splineToLinearHeading(autoConstants.closeTip.getPose(), autoConstants.closeTip.getH())
                .build();
        telemetry.addLine("Built preloadTraj");
        telemetry.update();

        closeBallTraj = drive
                .trajectorySequenceBuilder(preloadTraj.end())
                .splineToLinearHeading(autoConstants.closeBall.getPose(), autoConstants.closeBall.getH())
                .build();
        telemetry.addLine("Built closeBallTraj");
        telemetry.update();

        shoot1Traj = drive
                .trajectorySequenceBuilder(closeBallTraj.end())
                .splineToLinearHeading(autoConstants.closeTip.getPose(), autoConstants.closeTip.getH())
                .build();
        telemetry.addLine("Built shoot1Traj");
        telemetry.update();

        midBallTraj = drive
                .trajectorySequenceBuilder(shoot1Traj.end())
                .splineToLinearHeading(autoConstants.midBall.getPose(), autoConstants.midBall.getH())
                .build();
        telemetry.addLine("Built midBallTraj");
        telemetry.update();

        shoot2Traj = drive
                .trajectorySequenceBuilder(midBallTraj.end())
                .splineToLinearHeading(autoConstants.closeTip.getPose(), autoConstants.closeTip.getH())
                .build();
        telemetry.addLine("Built shoot2Traj");
        telemetry.update();

        farBallTraj = drive
                .trajectorySequenceBuilder(shoot2Traj.end())
                .splineToLinearHeading(autoConstants.farBall.getPose(), autoConstants.farBall.getH())
                .build();
        telemetry.addLine("Built farBallTraj");
        telemetry.update();

        shoot3Traj = drive
                .trajectorySequenceBuilder(farBallTraj.end())
                .splineToLinearHeading(autoConstants.closeTip.getPose(), autoConstants.closeTip.getH())
                .build();
        telemetry.addLine("Built shoot1Traj");
        telemetry.update();

        goHomeTraj = drive
                .trajectorySequenceBuilder(shoot3Traj.end())
                .setReversed(false)
                .splineToLinearHeading(autoConstants.START.getPose(), autoConstants.START.getH())
                .build();
        telemetry.addLine("Built goHomeTraj");
        telemetry.update();

        while (opModeInInit() && !isStopRequested()) {
            //drive.setPoseEstimate(autoConstants.START.getPose());
            drive.updatePoseEstimate();
            Pose2d p = drive.getPoseEstimate();
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.addData("drive heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }

        drive.setPoseEstimate(autoConstants.START.getPose());
        waitForStart();


        while (opModeIsActive() && !isStopRequested() && !commandMachine.hasCompleted()) {
            //shooter.shoot();
            drive.update();
            limelight.update();
            commandMachine.run(drive.isBusy() || commandBusy);
            telemetry.addData("target pose", targetPoint);
            telemetry.addData("drive x", drive.getPoseEstimate().getX());
            telemetry.addData("drive y", drive.getPoseEstimate().getY());
            telemetry.addData("drive heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
        limelight.stop();
        Thread.sleep(500);
    }
}
