package org.firstinspires.ftc.teamcode.HARDWARES;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.HARDWARES.UPPERPOWERFILE.upperpowerbound;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.SUBS.PowerSUB;
import org.firstinspires.ftc.teamcode.SUBS.SERVOSUB;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.initMovement.InitMovementController;

import java.util.List;

public class HARDWARECONFIG {
    boolean slowmode = false;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;
    public SERVOSUB servosub = null;
    public PowerSUB powersub = null;
    //public org.firstinspires.ftc.teamcode.SUBS.ARMSUB armSub = null;
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    double heading = 0;
    double distance = 0;
    Pose2d startPose = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    MecanumDrive drive = null;
    FtcDashboard dash = null;
    double x = 0;
    double y = 0;
    double indicator = 0;



    double color = 0;

    //private VisionPortal visionPortal;
    //private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private InitMovementController imc = null;

    ElapsedTime elapsedTime = null;

    public HARDWARECONFIG(LinearOpMode om, HardwareMap hwmap, Boolean auto) {
        initrobot(hwmap, om, auto);

        //powersub = new PowerSUB(hwmap);
        //servosub = new SERVOSUB(hwmap);

    }

    Action t = null;

    void initrobot(HardwareMap hwmap, LinearOpMode om, Boolean auto) {
        opMode = om;//
        telemetry = om.telemetry;
        imc = new InitMovementController(opMode.gamepad2,opMode.gamepad1);
        //clawsub = new CLAWSUB(hwmap);
        // armSub = new org.firstinspires.ftc.teamcode.SUBS.ARMSUB(hwmap, auto);
        frontLeftMotor = hwmap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hwmap.dcMotor.get("backLeftMotor");
        frontRightMotor = hwmap.dcMotor.get("frontRightMotor");
        backRightMotor = hwmap.dcMotor.get("backRightMotor");

        dash = FtcDashboard.getInstance();

//        gunmotorR.setDirection(DcMotorSimple.Direction.REVERSE);
//        gunmotorL.setDirection(DcMotorSimple.Direction.FORWARD);

        drive = new MecanumDrive(hwmap, (Pose2d) blackboard.getOrDefault(currentpose, new Pose2d(0, 0, 0)));

       // t = Turn(1.7);
//         limelight = hwmap.get(Limelight3A.class, "limelight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition,cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(585.459, 585.459, 326.896, 279.112)
                // ... these parameters are fx, fy, cx, cy.


                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hwmap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 60);



//

        elapsedTime = new ElapsedTime();
    }



    public static String currentpose = "currentpose";



    Action runningaction = null;



    public void indicatormath() {
        if (heading <= 0) {
            indicator = 1;
        } else if (heading > -0.6) {
            indicator = 0;
        }
    }





    public void buildtelemetry() {
        telemetry.addData("slowmode", slowmode);
        telemetry.addData("heading", heading);
        telemetry.addData("distance", distance);
        telemetry.addData("Power", upperpowerbound);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("indicator", indicator);
        //powersub.telemetry(telemetry);

        telemetry.update();
    }

    boolean touchpadwpressed = false;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 3, 13.875, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, 0, 0, 0);
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            }  else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }


    public void dobulk() {//
        imc.checkHasMovedOnInit();
        //heading = getheadingfromAT();
       // distance = getrangefromAT();
        double y = -opMode.gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = opMode.gamepad1.right_stick_x;
        boolean touchpadpressed = opMode.gamepad1.touchpad;
        if (touchpadpressed && !touchpadwpressed) {
            slowmode = !slowmode;
        }
        touchpadwpressed = touchpadpressed;
        double slowmodemultiplier = 0.8;


        touchpadwpressed = touchpadpressed;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double multiplier = 1;
        if (slowmode) {
            multiplier = slowmodemultiplier;
        }
        double frontLeftPower = ((y + x + rx) / denominator) * multiplier;
        double backLeftPower = ((y - x + rx) / denominator) * multiplier;
        double frontRightPower = ((y - x - rx) / denominator) * multiplier;
        double backRightPower = ((y + x - rx) / denominator) * multiplier;
//        double gunmotorPower = Range.clip(opMode.gamepad1.right_trigger, -1, 1);
//        double gunmotorPowerL = gunmotorPower;


        double armpower = 0;


//
//! -0.65, -0.4 0.5, 0.4 FIX
        if (heading >= -0.65 && heading <= -0.4) {
            indicator = 1;
        } else if (heading >= 0.4 && heading <= 0.5) {
            indicator = 1;
        } else {
            indicator = 0;
        }










//        if (opMode.gamepad1.left_bumper) {
//            powersub.intakereverse();
//        } else if (opMode.gamepad1.right_bumper) {
//            powersub.intakeonl();
//        } else {
//            powersub.intakeoff();
//        }
//        if (opMode.gamepad2.left_bumper){
//            powersub.intakereverseS();
//        }else if (opMode.gamepad2.right_bumper) {
//            powersub.intakeon();
//        }
//
//
//
//
//
//        if (opMode.gamepad2.left_trigger > 0) {
//            powersub.extrapower();
//
//        }else if (opMode.gamepad2.right_trigger > 0){
//            powersub.power();
//
//        }else {
//            powersub.gunidle();
//        }
//        if (opMode.gamepad2.a){
//            powersub.extralittle();
//
//        }
//
//        if (opMode.gamepad2.dpad_up){
//            servosub.SafetyON();
//        }
//        else if (opMode.gamepad2.dpad_down){
//            servosub.Safetyoff();
//        }










        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (imc.hasMovedOnInit()){
            //servosub.update();
            //powersub.update();
        }
        //armSub.update();

        buildtelemetry();

    }
    public Action Turn(double angle) {
        Scribe.getInstance().logData("here");
        return drive.actionBuilder(drive.localizer.getPose())
                .turnTo(Math.toRadians(angle + 15)).build();
    }
    public void lockit(double angle) {
        TelemetryPacket p = new TelemetryPacket();


//        Scribe.getInstance().logData(angle);
        if (angle != 0&& Math.abs(angle)>20) {
            Action t = Turn(Math.toRadians(angle));


            if (runningaction != null) {
//                runningaction.preview(p.fieldOverlay());
                Actions.runBlocking(runningaction);
                runningaction = null;
//                runningaction.run(p);
//                if (!runningaction.run(p)) {
//                    Scribe.getInstance().logData("true");
//                    runningaction = null;
//                }

            } else {
                runningaction = t;
            }
            dash.sendTelemetryPacket(p);
        }
    }
}

