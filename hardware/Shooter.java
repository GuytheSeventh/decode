package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.PIDController;
//maybe use external PID and calculate power

@Config
public class Shooter extends Mechanism {
    //private PIDController controller;

    public final DcMotorEx[] motors = new DcMotorEx[2];

    //private VoltageSensor voltage;

    public static double TICKS_PER_REV;
    public static double farShootRPM = 4500;
    public static double closeShootRPM = 3000;
    public static double farPwr = .8;
    public static double closePwr = .6;
    public static double passiveRPM = 500;
    public static double passPwr = .15;
    public static double rpmTolerance = 200;
    public static double kP = 4;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    private boolean far = true;

    public Shooter(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "leftShoot");
        motors[1] = hwMap.get(DcMotorEx.class, "rightShoot");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[0].setVelocityPIDFCoefficients(kP, kI, kD, kF);
        //motors[1].setVelocityPIDFCoefficients(kP, kI, kD, kF);
        TICKS_PER_REV = motors[0].getMotorType().getTicksPerRev();


    }

    public void shoot() {
        if (far) {
            double ticksPerSecond = farShootRPM * TICKS_PER_REV / 60.0;
            motors[0].setVelocity(ticksPerSecond);
            //motors[1].setVelocity(ticksPerSecond);
            motors[1].setPower(farPwr);
        }
        else{
            double ticksPerSecond = closeShootRPM * TICKS_PER_REV / 60.0;
            motors[0].setVelocity(ticksPerSecond);
            //motors[1].setVelocity(ticksPerSecond);
            motors[1].setPower(closePwr);
        }
    }
    public void unshoot(){
        double ticksPerSecond = closeShootRPM * TICKS_PER_REV / 60.0;
        motors[0].setVelocity(-ticksPerSecond);
        //motors[1].setVelocity(-ticksPerSecond);
        motors[1].setPower(-closePwr);
    }

    public void passivePower() {
        double ticksPerSecond = passiveRPM * TICKS_PER_REV / 60.0;
        motors[0].setVelocity(ticksPerSecond);
        //motors[1].setVelocity(ticksPerSecond);
        motors[1].setPower(passPwr);
    }

    public void stop(){
        motors[0].setVelocity(0);
        //motors[1].setVelocity(0);
        motors[1].setPower(0);
    }
    public boolean atTarget(){
        double shootRPM;
        if (far){
            shootRPM = farShootRPM;
        }
        else{
            shootRPM = closeShootRPM;
        }
        double currvol = motors[0].getVelocity();
        double idealvol = shootRPM * TICKS_PER_REV / 60.0;
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
        }
    }

}
