package org.firstinspires.ftc.teamcode.opmode.Tests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.reset();
        drive.setPoseEstimate(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS,0));
        waitForStart();

        TrajectorySequence traj =
        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .lineToSplineHeading(new Pose2d(0,50,PI/2))
                .lineToSplineHeading(new Pose2d(50,50,PI))
                .lineToSplineHeading(new Pose2d(50,0,3 * PI /2))
                .lineToSplineHeading(new Pose2d(0,0,0))
                .build();

        drive.followTrajectorySequence(traj);

    }
}
