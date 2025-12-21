package org.firstinspires.ftc.teamcode.opmode.Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class shootTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static double pwr = .8;
    private Shooter shoot;
    private Transfer transfer;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(this);
        shoot = new Shooter(this);
        transfer = new Transfer(this);
        intake.init(hardwareMap);
        transfer.init(hardwareMap);
        shoot.init(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
            shoot.shoot();
            intake.intake();
            transfer.run();

            telemetry.addData("Shoot rpm: ", shoot.getRpm());
            telemetry.addData("Close rpm: ", Shooter.closeShootRPM);
            telemetry.addData("Far rpm: ", Shooter.farShootRPM);
            telemetry.update();
        }
    }
