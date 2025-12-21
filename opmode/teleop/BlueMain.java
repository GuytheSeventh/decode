package org.firstinspires.ftc.teamcode.opmode.teleop;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlueMain", group = "_amain")
public class BlueMain extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this,false);
        robot.init(hardwareMap, false);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStopRequested() && !opModeIsActive()) {
            robot.telemetry(telemetry);
            telemetry.update();
            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.loop(gamepad1);
            robot.telemetry(telemetry);
            telemetry.update();
            idle();
        }

    }
}
