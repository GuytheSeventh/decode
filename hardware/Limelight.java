package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.opmode.auton.LimelightConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stuyfission.fissionlib.util.Mechanism;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends Mechanism {
    private static final double METERS_TO_INCHES = 39.3701;
    private static final long MAX_GLOBAL_STALENESS_MS = 150;

    private Limelight3A limelight;
    private final ArrayList<Location> locations = new ArrayList<>();

    // Last known global field pose from MegaTag2, in RoadRunner units (inches)
    private Pose2d globalPose = null;
    private long globalPoseStalenessMs = Long.MAX_VALUE;
    private boolean Red;

    public static class Location {
        public int tagID;
        public double x;     // left/right displacement (meters, robot space)
        public double y;     // forward/back displacement (meters, robot space)
        public double yaw;   // yaw angle (degrees, robot space)
        public double score;

        public double distScore;
        public double rotScore;
        public double yScore;

        public Location(int tagID, double x, double y, double yaw) {
            this.tagID = tagID;
            this.x = x;
            this.y = y;
            this.yaw = yaw;
        }
    }

    public static class DriveCommands {
        public double forward;  // meters * gain (robot forward)
        public double strafe;   // meters * gain (robot left/right)
        public double turn;     // degrees * gain (CCW positive)

        public DriveCommands(double forward, double strafe, double turn) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
        }
    }

    public Limelight(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);  // AprilTag pipeline
        limelight.start();
    }

    public void init(HardwareMap hwMap, boolean Red) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        if (Red) {
            limelight.pipelineSwitch(0);
        }
        else{
            limelight.pipelineSwitch(0);
        }
        this.Red = Red;
        limelight.start();
    }
    public boolean getColor(){
        return Red;
    }

    /**
     * Backwards-compatible update with no orientation info.
     * Prefer update(robotYawDeg) where possible.
     */
    public void update() {
        update(Double.NaN);
    }

    /**
     * Main update: feed robot yaw (deg) into MegaTag2, grab latest result,
     * update both per-tag relative locations and the global MegaTag2 pose.
     */
    public void update(double robotYawDeg) {
        if (limelight == null) return;

        // Feed IMU/heading into MegaTag2 if provided
        if (!Double.isNaN(robotYawDeg)) {
            limelight.updateRobotOrientation(robotYawDeg);
        }

        LLResult result = limelight.getLatestResult();
        locations.clear();

        if (result == null || !result.isValid()) {
            globalPose = null;
            globalPoseStalenessMs = Long.MAX_VALUE;
            return;
        }

        // ---------- GLOBAL FIELD POSE (MegaTag2) ----------
        Pose3D fieldPose = result.getBotpose_MT2();
        if (fieldPose != null) {
            Position fieldPos = fieldPose.getPosition();
            YawPitchRollAngles fieldAngles = fieldPose.getOrientation();

            if (fieldPos != null && fieldAngles != null) {
                double xMeters = fieldPos.x;
                double yMeters = fieldPos.y;
                double yawDeg = fieldAngles.getYaw(AngleUnit.DEGREES);

                double xInches = xMeters * METERS_TO_INCHES;
                double yInches = yMeters * METERS_TO_INCHES;
                double headingRad = Math.toRadians(yawDeg);

                globalPose = new Pose2d(xInches, yInches, headingRad);
                globalPoseStalenessMs = result.getStaleness();
            } else {
                globalPose = null;
                globalPoseStalenessMs = Long.MAX_VALUE;
            }
        } else {
            globalPose = null;
            globalPoseStalenessMs = Long.MAX_VALUE;
        }

        // ---------- PER-TAG ROBOT-SPACE LOCATIONS ----------
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) {
            return;
        }

        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag == null) continue;

            int id = tag.getFiducialId();

            Pose3D pose = tag.getTargetPoseRobotSpace();
            if (pose == null) continue;

            Position data = pose.getPosition();
            YawPitchRollAngles angles = pose.getOrientation();
            if (data == null || angles == null) continue;

            double x_m = data.x;  // left/right (robot space)
            double y_m = data.y;  // forward/back (robot space)
            // double z_m = data.z; // up/down, unused
            double yawDeg = angles.getYaw(AngleUnit.DEGREES);

            Location loc = new Location(id, x_m, y_m, yawDeg);
            locations.add(loc);
        }
    }

    /**
     * Get best AprilTag measurement in robot space using your weighted score.
     * Returns a dummy (-1,0,0,0) if none are currently visible.
     */
    public Location getBest() {
        if (locations.isEmpty()) {
            return new Location(-1, 0.0, 0.0, 0.0);
        }

        for (Location loc : locations) {
            loc.distScore = -X_WEIGHT * Math.abs(loc.x);
            loc.yScore    = -Y_WEIGHT * Math.abs(loc.y);
            loc.rotScore  = -ROT_WEIGHT * Math.abs(loc.yaw);
            loc.score     = loc.distScore + loc.yScore + loc.rotScore;
        }

        locations.sort((a, b) -> Double.compare(b.score, a.score));
        return locations.get(0);
    }

    /**
     * Compute simple P-controlled drive commands from the best tag.
     * Only call this if getBest().tagID >= 0.
     */
    public DriveCommands computeDriveCommands(double desiredFwdMeters,
                                              double forwardGain,
                                              double strafeGain,
                                              double turnGain) {
        Location best = getBest();
        double forward = (best.y - desiredFwdMeters) * forwardGain;
        double strafe  = best.x * strafeGain;
        double turn    = best.yaw * turnGain;
        return new DriveCommands(forward, strafe, turn);
    }

    /**
     * Latest global field pose from MegaTag2, in RoadRunner units (inches).
     * Returns null if no recent pose is available.
     */
    public Pose2d getGlobalPose() {
        if (globalPose == null) return null;
        if (globalPoseStalenessMs > MAX_GLOBAL_STALENESS_MS) return null;
        return globalPose;
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        Location best = getBest();
        telemetry.addData("Best Tag ID", best.tagID);
        telemetry.addData("Left/Right x (m)", "%.3f", best.x);
        telemetry.addData("Forward y (m)", "%.3f", best.y);
        telemetry.addData("Yaw (deg)", "%.2f", best.yaw);
        telemetry.addData("Score", "%.3f", best.score);
        telemetry.addData("Num Tags", locations.size());

        if (globalPose != null) {
            telemetry.addData("Global X (in)", "%.1f", globalPose.getX());
            telemetry.addData("Global Y (in)", "%.1f", globalPose.getY());
            telemetry.addData("Global H (deg)", "%.1f", Math.toDegrees(globalPose.getHeading()));
            telemetry.addData("Global Stale (ms)", globalPoseStalenessMs);
        } else {
            telemetry.addData("Global Pose", "none");
        }
        // Do NOT call telemetry.update() here; let the main OpMode handle it.
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public void setPipeline(int pipeline) {
        if (limelight != null) limelight.pipelineSwitch(pipeline);
    }
}
