package org.firstinspires.ftc.teamcode.SUBS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.caching.CacheManager;

import java.util.List;

@Config
public class LLcluster {
    private Limelight3A limelight;

    public static Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 3, 13.875, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0);

    CacheManager<Double> range = new CacheManager<>(this::getRange);

    public void init(HardwareMap hwMap) {
        // Initialize the official FTC Limelight hardware device
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // Set to AprilTag pipeline index
        limelight.start();

        // Note: cameraPosition and cameraOrientation offsets can also
        // be configured directly in the Limelight web UI (http://limelight.local:5801)
    }

    public void sendToDash() {
        // Limelight handles its own camera stream via http://limelight.local:5800.
        // This is kept empty to preserve your existing method calls/structure.
    }

    public double RANGE() {
        range.refreshCache();
        return range.loadCache();
    }

    public double getDistance() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose = fiducials.get(0).getCameraPoseTargetSpace();
                if (pose != null) {
                    // Convert Z depth meters to inches
                    return pose.getPosition().z * 39.3701;
                }
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    public double getRange() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose = fiducials.get(0).getCameraPoseTargetSpace();
                if (pose != null) {
                    // Mirrors your original getRange() which returned roll
                    return pose.getOrientation().getRoll();
                }
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    public double getBearing() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                return fiducials.get(0).getTargetXDegrees(); // Left/right angle error
            }
            return result.getTx();
        }
        return 0.0; // If no tag is seen, assume 0 offset
    }

    public boolean isTagDetected() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && !result.getFiducialResults().isEmpty();
    }

    public void telemetry(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                telemetry.addLine(String.format("\n==== Tag ID (%d)", fiducial.getFiducialId()));
                telemetry.addLine(String.format("Target X/Y (deg): %6.1f %6.1f", fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees()));

                org.firstinspires.ftc.robotcore.external.navigation.Pose3D pose = fiducial.getCameraPoseTargetSpace();
                if (pose != null) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            pose.getPosition().x * 39.3701,
                            pose.getPosition().y * 39.3701,
                            pose.getPosition().z * 39.3701));
                }
            }
        }

        telemetry.addLine(String.format("range %6.1f", range.loadCache()));
    }
}