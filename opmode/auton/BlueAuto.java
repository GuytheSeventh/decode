package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAuto", preselectTeleOp = "BlueMain")
public class BlueAuto extends auto {
    public BlueAuto() {
        super(false);
    }
}