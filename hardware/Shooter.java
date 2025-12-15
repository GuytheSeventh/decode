package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.PIDFController;
//maybe use external PID and calculate power

@Config
public class Shooter extends Mechanism {
    //private PIDController controller;
    PIDFController pidf;

    public final DcMotorEx[] motors = new DcMotorEx[2];

    //private VoltageSensor voltage;

    public static double TICKS_PER_REV = 100;
    public static double farShootRPM = 6000;
    public static double closeShootRPM = 5000;
    public static double farPwr = .9;
    public static double closePwr = .8;
    public static double passiveRPM = 500;
    public static double passPwr = .15;
    public static double rpmTolerance = 200;
    public static double kP = 2.0;
    public static double kI = 0.0;
    public static double kD = 0.5;
    public static double kF = 0.00008;
    private boolean far = true;
    private double ticksPerSecond;

    public Shooter(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        pidf = new PIDFController(kP, kI, kD, kF);
        motors[0] = hwMap.get(DcMotorEx.class, "leftShoot");
        motors[1] = hwMap.get(DcMotorEx.class, "rightShoot");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        pidf.setFeedForward(PIDFController.FeedForward.LINEAR);
        TICKS_PER_REV = motors[0].getMotorType().getTicksPerRev();


    }

    public void shoot() {
        pidf.setPIDF(kP, kI, kD, kF);
        if (far) {
            ticksPerSecond = farShootRPM * TICKS_PER_REV / 60.0;
            double currentTicks = motors[0].getVelocity();
            double output = pidf.calculate(currentTicks, ticksPerSecond);
            output = Math.max(-1, Math.min(1, output));
            motors[1].setPower(output);
            motors[0].setPower(output);
        }
        else{
            ticksPerSecond = closeShootRPM * TICKS_PER_REV / 60.0;
            double currentTicks = motors[0].getVelocity();
            double output = pidf.calculate(currentTicks, ticksPerSecond);
            output = Math.max(-1, Math.min(1, output));
            motors[1].setPower(output);
            motors[0].setPower(output);
        }
    }
    public void unshoot(){
        motors[0].setPower(-closePwr);
        //motors[1].setVelocity(-ticksPerSecond);
        motors[1].setPower(-closePwr);
    }

    public void passivePower() {
        motors[0].setPower(passPwr);
        //motors[1].setVelocity(ticksPerSecond);
        motors[1].setPower(passPwr);
    }

    public void stop(){
        motors[0].setPower(0);
        motors[1].setPower(0);
    }
    public boolean atTarget(){
        double currvol = motors[0].getVelocity();
        double idealvol = (!far ? closeShootRPM : farShootRPM) * TICKS_PER_REV / 60.0;
        return Math.abs(currvol - idealvol) < rpmTolerance;
    }
    public void setFar(boolean far){
        this.far = far;
    }




    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            shoot();

        } else{
            passivePower();
            pidf.clearTotalError();
        }
    }

}
