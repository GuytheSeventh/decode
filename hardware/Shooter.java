package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.util.Mechanism;


import org.firstinspires.ftc.teamcode.util.PIDController;

@Config
public class Shooter extends Mechanism {
    private PIDController controller;

    public final DcMotorEx[] motors = new DcMotorEx[2];

    private VoltageSensor voltage;

    public static double TICKS_PER_REV = 28.0;   // goBILDA 6000 RPM (1:1)
    public static double shootRPM = 4500;
    public static double passiveRPM = 100;
    public static double rpmTolerance = 200;
    public static double shootFollowerPower = 1.0;
    public static double passiveFollowerPower = 0.2;
    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static final double NOMINAL_VOLTAGE = 12.0;

    public Shooter(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        //voltage = hwMap.voltageSensor.iterator().next();
       // servo = hwMap.get(Servo.class, "shooterServo");

        //servo.setPosition(srvo);
        motors[0] = hwMap.get(DcMotorEx.class, "leftShoot");
        motors[1] = hwMap.get(DcMotorEx.class, "rightShoot");

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // might be wrong RunMode
        motors[1].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[0].setVelocityPIDFCoefficients(kP, kI, kD, kF);

    }

    public void shoot() {
        double ticksPerSecond = shootRPM * TICKS_PER_REV / 60.0;
        motors[0].setVelocity(ticksPerSecond);
        motors[1].setPower(shootFollowerPower);
    }

    public void passivePower() {
        double ticksPerSecond = passiveRPM * TICKS_PER_REV / 60.0;
        motors[0].setVelocity(ticksPerSecond);
        motors[1].setPower(passiveFollowerPower);
    }

    public void stop(){
        motors[0].setPower(0);
        motors[1].setPower(0);
    }
    public boolean atTarget(){
        double currvol = motors[0].getVelocity();
        double idealvol = shootRPM * TICKS_PER_REV / 60.0;
        return Math.abs(currvol - idealvol) < rpmTolerance;
    }

    public void adjustPower(){
//gonna be the auto power adjust but limelight needs to be done (and tested) first
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
