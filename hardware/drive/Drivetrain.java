package org.firstinspires.ftc.teamcode.hardware.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class Drivetrain extends Mechanism {

    private SampleMecanumDrive rrDrive;
    public GoBildaPinpointDriver odo;

    public Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        rrDrive = new SampleMecanumDrive(hwMap);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.odo = rrDrive.odo ;
    }

    // ---------- POSE ----------

    /** Current robot pose estimate from Road Runner. */
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
    public void setWeightedDrivePower(Pose2d drivePower) {
        rrDrive.setWeightedDrivePower(drivePower);
    }

    /** Update RoadRunner's localization and trajectory follower. */
    public void update() {
        rrDrive.update();
    }

    @Override
    public void loop(Gamepad gamepad) {
        // Default tele-op drive (unused if Scoring drives instead)
        setDrivePower(new Pose2d(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
        ));
        update();
    }

    // ---------- TELEMETRY ----------

    public void telemetry(Telemetry telemetry) {
        Pose2d pose = getPoseEstimate();
        telemetry.addData("Drive X", pose.getX());
        telemetry.addData("Drive Y", pose.getY());
        telemetry.addData("Drive H", pose.getHeading());
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
