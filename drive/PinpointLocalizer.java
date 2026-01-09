
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


/**
 * Road Runner localizer wrapper for goBILDA Pinpoint.
 * Converts Pinpoint's millimeter outputs into Road Runner's inch units.
 */
public class PinpointLocalizer implements Localizer {

    private static final double MM_TO_IN = 1.0 / 25.4;

    private final GoBildaPinpointDriver pinpoint;
    private Pose2d poseEstimate = new Pose2d();
    private Pose2d poseVelocity = new Pose2d();

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose) {
        poseEstimate = pose;

        // Sync Pinpoint internal state to Road Runner
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()
        ));
    }

    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        pinpoint.update();

        poseEstimate = new Pose2d(
                pinpoint.getPosX() * MM_TO_IN,
                pinpoint.getPosY() * MM_TO_IN,
                pinpoint.getHeading()
        );

        poseVelocity = new Pose2d(
                pinpoint.getVelX() * MM_TO_IN,
                pinpoint.getVelY() * MM_TO_IN,
                pinpoint.getHeadingVelocity()
        );
    }
}
