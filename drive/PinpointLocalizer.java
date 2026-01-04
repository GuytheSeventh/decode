package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

public class PinpointLocalizer implements Localizer {
    private GoBildaPinpointDriver odo;

    public PinpointLocalizer(GoBildaPinpointDriver odo) {
        this.odo = odo;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        Pose2D p = odo.getPosition();

        double x = p.getY(DistanceUnit.INCH);      // forward
        double y = -p.getX(DistanceUnit.INCH);     // left
        double heading = p.getHeading(AngleUnit.RADIANS);

        return new Pose2d(x, y, heading);
    }


    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, pose2d.getX(), pose2d.getY(), AngleUnit.RADIANS, pose2d.getHeading()));

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        Pose2D v = odo.getVelocity();

        double x = v.getY(DistanceUnit.INCH);
        double y = -v.getX(DistanceUnit.INCH);
        double heading = v.getHeading(AngleUnit.RADIANS);

        return new Pose2d(x, y, heading);
    }



    @Override
    public void update() {
        odo.update();
        FtcDashboard.getInstance().getTelemetry().addData("status", odo.getDeviceStatus());

    }
}
