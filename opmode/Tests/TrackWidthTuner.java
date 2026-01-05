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
public class TrackWidthTuner extends LinearOpMode {
    public GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        //odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.RADIANS,0));
        while (opModeInInit() && !isStopRequested()) {
            odo.update();
            Pose2D p = odo.getPosition();
            telemetry.addData("x", p.getX(DistanceUnit.INCH));
            telemetry.addData("y", p.getY(DistanceUnit.INCH));
            telemetry.addData("heading", Math.toDegrees(p.getHeading(AngleUnit.RADIANS)));
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            odo.update();
            Pose2D p = odo.getPosition();
            telemetry.addData("x", p.getX(DistanceUnit.INCH));
            telemetry.addData("y", p.getY(DistanceUnit.INCH));
            telemetry.addData("heading", Math.toDegrees(p.getHeading(AngleUnit.RADIANS)));
            telemetry.update();
        }

    }
}
