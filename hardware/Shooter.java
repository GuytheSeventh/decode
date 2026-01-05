package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.stuyfission.fissionlib.util.Mechanism;
import org.firstinspires.ftc.teamcode.util.PIDFController;
//maybe use external PID and calculate power

@Config
public class Shooter extends Mechanism {
    public final DcMotorEx[] motors = new DcMotorEx[2];
    VoltageSensor batteryVoltageSensor =
            opMode.hardwareMap.voltageSensor.iterator().next();



    public static double TICKS_PER_REV = 28;

    public static double farShootRPM   = 5000;
    public static double closeShootRPM = 4500;
    public static double farPwr        = 0.9;   // follower power
    public static double closePwr      = 0.8;

    public static double passiveRPM = 500;
    public static double passPwr    = 0.15;

    public static double rpmTolerance = 200;    // RPM

    // Velocity PIDF for encoder motor (motors[0])
    public static double kP = 0.0005;

    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 1.0/2800; // starting point, see below
    private final PIDFController pid = new PIDFController(kP,kI,kD,kF);

    private boolean far = false;

    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "leftShoot");   // encoder motor
        motors[1] = hwMap.get(DcMotorEx.class, "rightShoot");  // follower

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[1].setDirection(DcMotor.Direction.FORWARD); // default

    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public void shoot() {
        pid.setPIDF(kP, kI, kD, kF);

        double targetRpm = far ? farShootRPM : closeShootRPM;
        double targetTps = rpmToTicksPerSecond(targetRpm);

        pid.setSetPoint(targetTps);

        double currentTps = motors[0].getVelocity();
        double output = pid.calculate(currentTps);

        double voltage = batteryVoltageSensor.getVoltage();
        output *= 12.0 / voltage;

        output = Math.max(-1.0, Math.min(1.0, output));


        motors[0].setPower(output);
        motors[1].setPower(output);
    }


    public void unshoot() {
        double tps = rpmToTicksPerSecond(closeShootRPM);
        motors[0].setVelocity(-tps);
        motors[1].setPower(-closePwr);
    }

    public double getRpm() {
        double tps = motors[0].getVelocity(); // ticks per second
        return tps * 60.0 / TICKS_PER_REV;
    }

    public void passivePower() {
        double tps = rpmToTicksPerSecond(passiveRPM);
        motors[0].setVelocity(tps);
        motors[1].setPower(passPwr);
    }

    public void stop() {
        motors[0].setVelocity(0);
        motors[1].setPower(0);
    }

    public boolean atTarget() {
        double targetRpm = far ? farShootRPM : closeShootRPM;
        double currentRpm = getRpm();
        return Math.abs(currentRpm - targetRpm) < rpmTolerance;
    }

    public void setFar(boolean far) {
        this.far = far;
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            shoot();
        } else {
            passivePower();
        }
    }
}
