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

@Config
public class Shooter extends Mechanism {

    // ---------------- HW ----------------
    public final DcMotorEx[] motors = new DcMotorEx[2];

    /** The motor that actually has the encoder PLUGGED IN. */
    private static final int ENCODER_MOTOR_INDEX = 0;

    private VoltageSensor battery;

    // ---------------- Tunables ----------------
    // Your 6000 RPM Yellow Jacket: start with kF ~ 1/6000 = 0.0001667
    // Then tune kP small. Your old kP=4 was for a totally different scale.
    public static double farShootRPM = 6000;
    public static double closeShootRPM = 5000;

    public static double passPwr = 0.15;
    public static double rpmTolerance = 200;

    // PIDF over RPM -> motor power
    public static double kP = .0002;
    public static double kI = 0.0;
    public static double kD = .00005;
    public static double kF = (1/6000.0);

    public static double maxPower = 1;
    public static double nominalVoltage = 12.0;
    public static boolean useVoltageComp = false;

    // If you *really* want to override ticks/rev for math/telemetry, set this > 0.
    // Otherwise we pull it from motorType.

    // ---------------- State ----------------
    private PIDFController pidf;
    private boolean far = true;

    public static double ticksPerRev = 1.0;

    public Shooter(LinearOpMode opMode) { this.opMode = opMode; }

    @Override
    public void init(HardwareMap hwMap) {
        motors[0] = hwMap.get(DcMotorEx.class, "leftShoot");
        motors[1] = hwMap.get(DcMotorEx.class, "rightShoot");

        // Battery for feedforward compensation
        battery = hwMap.voltageSensor.iterator().next();

        // IMPORTANT: set directions so both wheels spin the same "shooting" direction.
        // Keep what you had, but verify physically.
        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.FORWARD);

        motors[0].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motors[1].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We are doing software PIDF, so run BOTH open-loop.
        // You can still read encoder velocity in RUN_WITHOUT_ENCODER.
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ticksPerRev = motors[ENCODER_MOTOR_INDEX].getMotorType().getTicksPerRev();

        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setFeedForward(PIDFController.FeedForward.LINEAR);
        pidf.setIntegrationBounds(-0.25, 0.25); // keep I from winding up


    }

    /**
     * Call this repeatedly (every loop) while shooting.
     * In your Scoring code, you already do this: if (shoot) shooter.shoot();
     * So this method can "do everything" without a separate update() call.
     */
    public void shoot() {
        // Refresh tunables live (dashboard edits)
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setFeedForward(PIDFController.FeedForward.LINEAR);

        double targetRpm = far ? farShootRPM : closeShootRPM;
        double currentRpm = getRpm();

        // PIDFController.calculate(pv, sp): pv=current, sp=target
        double power = pidf.calculate(currentRpm, targetRpm);

        // Optional voltage compensation so behavior stays similar as battery drops
        if (useVoltageComp && battery != null) {
            double v = battery.getVoltage();
            if (v > 1.0) power *= (nominalVoltage / v);
        }
        PIDFCoefficients pid = new PIDFCoefficients(kP,kI,kD,kF);

        motors[0].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pid);
        motors[0].setVelocity(targetRpm * ticksPerRev / 60.0);
        motors[1].setPower(power);

       // setShooterPower(power);
    }
    public void initShoot(){
        motors[0].setPower(.5);
        motors[1].setPower(.5);
    }

    public void setFar(boolean far) {
        // if switching setpoints mid-shot, don’t keep integral from previous target
        if (this.far != far) {
            pidf.reset();
        }
        this.far = far;
    }

    public void passivePower() {
        pidf.reset();
        setShooterPower(passPwr);
    }

    public void stop() {
        pidf.reset();
        setShooterPower(0);
    }

    public void unshoot() {
        pidf.reset();
        setShooterPower(-0.3);
    }

    /** Returns encoder motor velocity in ticks/sec (for your existing telemetry). */
    public double getVelocity() {
        return motors[ENCODER_MOTOR_INDEX].getVelocity();
    }

    /** Returns measured RPM from the motor that actually has the encoder. */
    public double getRpm() {
        double ticksPerSecond = motors[ENCODER_MOTOR_INDEX].getVelocity();
        return (ticksPerSecond * 60.0) / ticksPerRev;
    }

    public boolean atTarget() {
        double targetRpm = far ? farShootRPM : closeShootRPM;
        return Math.abs(getRpm() - targetRpm) <= rpmTolerance;
    }

    private void setShooterPower(double pwr) {
        double clipped = clamp(pwr, -maxPower, maxPower);
        motors[0].setPower(clipped);
        motors[1].setPower(clipped);
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
    public double currRPM(){
        return motors[0].getVelocity();
    }

    @Override
    public void loop(Gamepad gamepad) {
        // Optional: keep blank, since Scoring is already calling shoot()/stop()/passivePower()
        // This exists only because Mechanism has it.
    }
}
