package org.firstinspires.ftc.teamcode.SUBS;

import static java.lang.Math.atan2;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.caching.CacheManager;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.vision.LensIntrinsics;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.vision.LensIntrinsicsImpl;

import java.util.List;
import java.util.function.Function;

@Config
public class Cluster {
    static final int VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY = 0xC45;
    static final int PRODUCT_ID_ARDUCAM_OV5648 = 0x6366;



    public static Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 3, 13.875, 0);
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    CacheManager<Double> range = new CacheManager<>(this::getRange);
    public static LensIntrinsics intrs = new LensIntrinsicsImpl(585.459,585.459,326.896,279.112);

    public void init(HardwareMap hwMap){
        CameraCompatibilityManager.getInstance()
                .addQuirk(
                        VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY,
                        PRODUCT_ID_ARDUCAM_OV5648,
                        CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE);
        aprilTag = new AprilTagProcessor.Builder()


                .setCameraPose(cameraPosition,cameraOrientation)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(intrs.getFx(),intrs.getFy(),intrs.getCx(),intrs.getCy())
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        //builder.setCameraResolution(Size cameraResolution = new Size(640, 480));
        aprilTag.setDecimation(100);
        visionPortal = builder.build();
    }
    public void sendToDash(){
        FtcDashboard.getInstance().startCameraStream(visionPortal,30.0);
    }
    public double RANGE(){
        range.refreshCache();
        return range.loadCache();
    }

    public double getRange(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                return clusterDet.ftcPose.roll;
            }
        }
        return Double.POSITIVE_INFINITY;
    }

    public void telemetry(Telemetry telemetry){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                return clusterDet.ftcPose.roll;
            }
        }


        telemetry.addLine(String.format("range %6.1f",  range.loadCache()));

    }
}
