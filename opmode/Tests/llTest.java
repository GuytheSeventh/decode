
package org.firstinspires.ftc.teamcode.opmode.Tests;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Transfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "llTest", group = "drive")
public class llTest extends LinearOpMode {


    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(this);
        intake.init(hardwareMap);
        transfer = new Transfer(this);
        transfer.init(hardwareMap);
        shooter = new Shooter(this);
        shooter.init(hardwareMap);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStopRequested() && !opModeIsActive()) {
            shooter.shoot();
            intake.telemetry(telemetry);
            transfer.telemetry(telemetry);
            shooter.telemetry(telemetry);
            telemetry.update();
            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            intake.intake();
            transfer.run();
            shooter.shoot();
            intake.telemetry(telemetry);
            transfer.telemetry(telemetry);
            shooter.telemetry(telemetry);

            double rpm1 = shooter.motors[0].getVelocity();
            double rpm2 = shooter.motors[1].getVelocity();
            double rpm = Math.max(rpm1,rpm2);
            double currentRpm = rpm * 60.0 / 28;
            telemetry.addData("Shooter RPM", currentRpm);
            telemetry.addData("Ideal Close RPM", Shooter.closeShootRPM);
            telemetry.addData("Ideal Far RPM", Shooter.farShootRPM);
            telemetry.update();
            idle();
        }

    }
}
