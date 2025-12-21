
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.stuyfission.fissionlib.util.Mechanism;

public class Robot extends Mechanism {
    private ElapsedTime elapsedTime;
    public Scoring scoring;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        scoring = new Scoring(opMode);
    }
    public Robot(LinearOpMode opMode, boolean Red) {
        this.opMode = opMode;
        scoring = new Scoring(opMode, Red);
    }

    @Override
    public void init(HardwareMap hwMap) {
        scoring.init(hwMap);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }
    public void init(HardwareMap hwMap, boolean Red) {
        scoring.init(hwMap, Red);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        scoring.telemetry(telemetry);
        telemetry.addData("loop time (ms)", elapsedTime.milliseconds());
        elapsedTime.reset();
        telemetry.update();
    }

    @Override
    public void loop(Gamepad gamepad) {
        scoring.loop(gamepad);
    }
}
