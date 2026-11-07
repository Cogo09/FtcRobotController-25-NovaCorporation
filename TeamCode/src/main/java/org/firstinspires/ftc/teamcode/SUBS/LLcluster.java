package org.firstinspires.ftc.teamcode.SUBS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.caching.CacheManager;

import java.util.List;

@Config
public class LLcluster {
    private Limelight3A limelight;
    private LLResult latestResult = null; // Cached once per loop

    public static Position cameraPosition = new Position(DistanceUnit.INCH, 0, 3, 13.875, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    CacheManager<Double> range = new CacheManager<>(this::calculateRange);

    public void init(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // Make sure your AprilTag pipeline is in slot 0 in the Web UI!
        limelight.start();
    }

    // Call this ONCE per loop inside HARDWARECONFIG.dobulk()
    public void update() {
        if (limelight != null) {
            latestResult = limelight.getLatestResult();
            range.refreshCache();
        }
    }

    public void sendToDash() {
        // Left empty for dashboard config integration
    }

    public double RANGE() {
        return range.loadCache();
    }

    private double calculateRange() {
        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                Pose3D pose = fiducials.get(0).getCameraPoseTargetSpace();
                if (pose != null && pose.getPosition().z != 0) {
                    // Use Math.abs because TargetSpace Z is negative in front of the tag
                    return Math.abs(pose.getPosition().z) * 39.3701;
                }
            }
        }
        return 0.0;
    }

    public double getDistance() {
        return calculateRange();
    }

    public double getRange() {
        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                Pose3D pose = fiducials.get(0).getCameraPoseTargetSpace();
                if (pose != null) {
                    return pose.getOrientation().getRoll();
                }
            }
        }
        return 0.0;
    }

    public double getBearing() {
        if (latestResult != null && latestResult.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
            if (!fiducials.isEmpty()) {
                return -fiducials.get(0).getTargetXDegrees();
            }
            return -latestResult.getTx();
        }
        return -0.0;
    }

    public boolean isTagDetected() {
        return latestResult != null && latestResult.isValid() && !latestResult.getFiducialResults().isEmpty();
    }

    public void telemetry(Telemetry telemetry) {
        if (limelight == null) {
            telemetry.addData("Limelight", "NOT INITIALIZED");
            return;
        }

        // Always fetch a fresh result when building telemetry
        latestResult = limelight.getLatestResult();

        LLStatus status = limelight.getStatus();
        telemetry.addData("LL Connected", limelight.isConnected());
        telemetry.addData("LL Pipeline", "Index: %d | Type: %s", status.getPipelineIndex(), status.getPipelineType());

        if (latestResult == null) {
            // If null, start() was not called or polling stopped
            limelight.start();
            telemetry.addData("Limelight Status", "Result is NULL (Calling start()...)");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        telemetry.addData("LL isValid()", latestResult.isValid());
        telemetry.addData("LL Fiducial Count", fiducials.size());
        telemetry.addData("LL Staleness (ms)", latestResult.getStaleness());

        if (latestResult.isValid() || !fiducials.isEmpty()) {
            telemetry.addData("Limelight Status", "TARGET LOCKED");
            telemetry.addData("LL tx / ty (deg)", "%.2f / %.2f", latestResult.getTx(), latestResult.getTy());

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                telemetry.addLine(String.format("==== Tag ID (%d) ====", fiducial.getFiducialId()));
                telemetry.addLine(String.format("Target X/Y (deg): %6.1f %6.1f", fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees()));

                Pose3D pose = fiducial.getCameraPoseTargetSpace();
                if (pose != null) {
                    telemetry.addLine(String.format("XYZ (inch): %6.1f %6.1f %6.1f",
                            pose.getPosition().x * 39.3701,
                            pose.getPosition().y * 39.3701,
                            Math.abs(pose.getPosition().z) * 39.3701));
                }
            }
        } else {
            telemetry.addData("Limelight Status", "No valid target in frame");
        }

        telemetry.addData("Cached Range (in)", "%.2f", range.loadCache());
    }
}