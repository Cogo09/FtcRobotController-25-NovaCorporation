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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.caching.CacheManager;

import java.util.List;

@Config
public class LLcluster {
    private Limelight3A limelight;

    public static Position cameraPosition = new Position(DistanceUnit.INCH, 0, 3, 13.875, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    CacheManager<Double> range = new CacheManager<>(this::calculateRange);

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // Ensure pipeline 0 is set to AprilTag in the Limelight web UI
        limelight.start();
    }

    public void sendToDash() {
        // Left empty for dashboard config integration
    }

    public double RANGE() {
        range.refreshCache();
        return range.loadCache();
    }

    private double calculateRange() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                // Use getCameraPoseTargetSpace() instead of non-existent camera space methods
                Pose3D pose = fiducials.get(0).getCameraPoseTargetSpace();
                if (pose != null) {
                    // Convert meters to inches
                    return pose.getPosition().z * 39.3701;
                }
            }
        }
        return 0.0;
    }

    public double getDistance() {
        return calculateRange();
    }

    public double getRange() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                Pose3D pose = fiducials.get(0).getCameraPoseTargetSpace();
                if (pose != null) {
                    // Access orientation or roll safely from target space pose
                    return pose.getOrientation().getRoll();
                }
            }
        }
        return 0.0;
    }

    public double getBearing() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                return fiducials.get(0).getTargetXDegrees(); // Left/right error in degrees
            }
            return result.getTx();
        }
        return 0.0;
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
                telemetry.addLine(String.format("==== Tag ID (%d) ====", fiducial.getFiducialId()));
                telemetry.addLine(String.format("Target X/Y (deg): %6.1f %6.1f", fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees()));

                Pose3D pose = fiducial.getCameraPoseTargetSpace();
                if (pose != null) {
                    telemetry.addLine(String.format("XYZ (inch): %6.1f %6.1f %6.1f",
                            pose.getPosition().x * 39.3701,
                            pose.getPosition().y * 39.3701,
                            pose.getPosition().z * 39.3701));
                }
            }
        } else {
            telemetry.addData("Limelight", "No valid target");
        }

        telemetry.addData("Cached Range", range.loadCache());
    }
}