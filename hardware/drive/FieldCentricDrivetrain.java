package org.firstinspires.ftc.teamcode.hardware.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class FieldCentricDrivetrain extends Mechanism {

    SampleMecanumDrive rrDrive;
    public GoBildaPinpointDriver odo;

    public FieldCentricDrivetrain(LinearOpMode opMode) { this.opMode = opMode; }

    public Pose2d getPoseEstimate() {
        return rrDrive.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose){
        rrDrive.setPoseEstimate(pose);
    }

    /** For backwards compatibility if you used currPos() before. */
    public Pose2d currPos() {
        return getPoseEstimate();
    }

    // ---------- DRIVING ----------

    /** Directly set drive power (x = fwd, y = strafe, heading = turn). */
    public void setDrivePower(Pose2d drivePower) {
        rrDrive.setWeightedDrivePower(drivePower);
    }

    /** Update RoadRunner's localization and trajectory follower. */
    public void update() {
        rrDrive.update();
    }

    @Override
    public void init(HardwareMap hwMap) {
        rrDrive = new SampleMecanumDrive(hwMap);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.odo = rrDrive.odo;

    }

    public void loop(Gamepad gamepad) {
        double y = -gamepad.left_stick_y; // Remember, this is reversed!
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.

        double botHeading = rrDrive.getHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        rrDrive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
    }
    public void setMotorPowers(double fL, double bL, double bR, double fR){
        rrDrive.setMotorPowers(fL,bL,bR,fR);
    }
    public double getHeading(){
        return rrDrive.getHeading();
    }
    public void resetIMU(){
        rrDrive.resetIMU();
    }
}
