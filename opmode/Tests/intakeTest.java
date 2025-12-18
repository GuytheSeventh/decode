package org.firstinspires.ftc.teamcode.opmode.Tests;

import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Intake;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.input.GamepadStatic;

import org.firstinspires.ftc.teamcode.opmode.teleop.Controls;

@TeleOp(name = "intakeTest", group = "drive")
public class intakeTest extends LinearOpMode {
    Transfer transfer = new Transfer(this);
    Intake intake = new Intake(this);

    MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
    Gamepad gamepad = gamepad1;
    public void runOpMode() throws InterruptedException {
        transfer.init(hardwareMap);
        intake.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Ball: ", transfer.hasBall());
            telemetry.addData("Distance: ", transfer.ballFar());
            telemetry.update();
            if (GamepadStatic.isButtonPressed(gamepad1, Controls.INTAKE)) {
                intake.intake();
                if(transfer.hasBall()) {
                    transfer.stop();
                }
                else{
                    transfer.intake();
                }
            }
            else if (GamepadStatic.isButtonPressed(gamepad1, Controls.OUTTAKE)) {
                intake.setOut(intake.getOut() + .1);
                intake.outtake();
                transfer.backup();
            }
            else if (GamepadStatic.isButtonPressed(gamepad1, Controls.TRANSFER)){
                transfer.run();
            }
            else if (GamepadStatic.isButtonPressed(gamepad1, Controls.UNTRANSFER)){
                transfer.backup();
            }
            else{
                intake.resetOut();
                intake.stop();
                if (!(GamepadStatic.isButtonPressed(gamepad1, Controls.INTAKE)||
                        GamepadStatic.isButtonPressed(gamepad1,Controls.TRANSFER)||
                        GamepadStatic.isButtonPressed(gamepad1,Controls.OUTTAKE)||
                        GamepadStatic.isButtonPressed(gamepad1,Controls.UNTRANSFER))){
                    transfer.stop();
                }
            }
        }
    }
}
