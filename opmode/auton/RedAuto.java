package org.firstinspires.ftc.teamcode.opmode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAuto", preselectTeleOp = "RedMain")
public class RedAuto extends auto {
    public RedAuto() {
        super(true);
    }
}