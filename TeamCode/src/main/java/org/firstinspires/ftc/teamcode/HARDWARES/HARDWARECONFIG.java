package org.firstinspires.ftc.teamcode.HARDWARES;

import static com.qualcomm.robotcore.eventloop.opmode.OpMode.blackboard;
import static org.firstinspires.ftc.teamcode.HARDWARES.UPPERPOWERFILE.upperpowerbound;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SUBS.LLcluster;
import org.firstinspires.ftc.teamcode.SUBS.PowerSUB;
import org.firstinspires.ftc.teamcode.SUBS.SHOOTERSUB;
import org.firstinspires.ftc.teamcode.SUBS.Servosub;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.Scribe;
import org.gentrifiedApps.gentrifiedAppsUtil.controllers.initMovement.InitMovementController;

public class HARDWARECONFIG {
    LLcluster cluster = new LLcluster();
    //Cluster cluster = new Cluster();
    boolean slowmode = false;
    Telemetry telemetry = null;
    LinearOpMode opMode = null;
    public SHOOTERSUB shootersub = null;
    public PowerSUB powersub = null;
    public Servosub servos = null;
    //public org.firstinspires.ftc.teamcode.SUBS.ARMSUB armSub = null;
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    Limelight3A limelight = null;
    public boolean isRedAlliance = true; // Default to Red
    private boolean allianceButtonPreviouslyPressed = false;

    public LLResult limelightResult = null;


    double heading = 0;
    double distance = 0;
    Pose2d startPose = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    MecanumDrive drive = null;
    FtcDashboard dash = null;
    double x = 0;
    double y = 0;
    double indicator = 0;
    public boolean aimbot = false;


    double color = 0;

    //private VisionPortal visionPortal;
    //private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private InitMovementController imc = null;

    ElapsedTime elapsedTime = null;

    public HARDWARECONFIG(LinearOpMode om, HardwareMap hwmap, Boolean auto) {
        initrobot(hwmap, om, auto);

        powersub = new PowerSUB(hwmap);

        shootersub = new SHOOTERSUB();
        shootersub.init(hwmap);

    }


    Action t = null;

    void initrobot(HardwareMap hwmap, LinearOpMode om, Boolean auto) {
        opMode = om;//
        telemetry = om.telemetry;
        imc = new InitMovementController(opMode.gamepad2, opMode.gamepad1);
        servos = new org.firstinspires.ftc.teamcode.SUBS.Servosub(hwmap);
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


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cluster.init(hwmap);
        cluster.sendToDash();


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

    public double getrange() {
        return cluster.getDistance();
    }

    public void sendDashboardPacket() {
        TelemetryPacket packet = new TelemetryPacket();

        // Pass numerical variables to the packet for live graphing
        packet.put("Limelight Distance (in)", cluster.getDistance());
        packet.put("Limelight Bearing (deg)", cluster.getBearing());
        packet.put("Target Tag Detected", cluster.isTagDetected() ? 1 : 0);

        // Dispatch the packet to the dashboard server
        dash.sendTelemetryPacket(packet);
    }


    public void buildtelemetry() {
        telemetry.addData("slowmode", slowmode);
        telemetry.addData("heading", heading);
        telemetry.addData("distance", distance);
        telemetry.addData("Power", upperpowerbound);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("indicator", indicator);
        telemetry.addData("bearing", cluster.getBearing());
        // --- Limelight Telemetry Section ---

        cluster.telemetry(telemetry); // Calls LLcluster's built-in telemetry!
        shootersub.telemetry(telemetry);
        powersub.telemetry(telemetry);

        telemetry.update();
    }

    boolean touchpadwpressed = false;


    public void dobulk() {//
        imc.checkHasMovedOnInit();
        cluster.update();
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
        double slowmodemultiplier = 0.5;
        // 1. Alliance Toggle Button (Press Gamepad 1 Back button to switch Red <-> Blue)
        boolean allianceButtonPressed = opMode.gamepad2.rightStickButtonWasPressed();
        if (allianceButtonPressed && !allianceButtonPreviouslyPressed) {
            isRedAlliance = !isRedAlliance;
        }
        allianceButtonPreviouslyPressed = allianceButtonPressed;

        // Display current alliance on Driver Station telemetry
        telemetry.addData("Alliance Mode", isRedAlliance ? "RED ALLIANCE" : "BLUE ALLIANCE");


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
        if (opMode.gamepad2.right_trigger > 0) {
            powersub.extrapower();
        } else if (opMode.gamepad2.left_trigger > 0) {
            powersub.power();
        } else {
            powersub.gunidle();
        }
        if (opMode.gamepad1.left_trigger > 0) {
            servos.setRampUP();
        }
        if (opMode.gamepad1.right_trigger > 0) {
            servos.setRampDOWN();
        }
        if (opMode.gamepad2.right_bumper) {
            powersub.allon();
        } else if (opMode.gamepad1.right_bumper) {
            powersub.fronton();
        } else if (opMode.gamepad1.left_bumper) {
            powersub.intakereverse();
        } else if (opMode.gamepad2.left_bumper) {
            powersub.intakereverse();
        }else {
            powersub.intakeidle();
        }
//        if (opMode.gamepad1.dpad_up) {
//            shootersub.shootlock(cluster.getRange());
//        }
        if (opMode.gamepad2.dpad_down) {
            shootersub.shootunlock();
            shootersub.shootlock(280);
        }
        if (opMode.gamepad2.dpad_left) {
            shootersub.shootunlock();
            shootersub.shootlock(180.0);
        }
        if (opMode.gamepad2.dpad_right) {
            shootersub.shootunlock();
            shootersub.shootlock(350.0);
        }
        if (opMode.gamepad2.a) {
            shootersub.shootunlock();
            shootersub.shootlock(90.0);
        }
        if (opMode.gamepad2.x) {
            servos.setVertBOTTOM();
        }


//        if (opMode.gamepad1.left_stick_button){
//            shootersub.setVert(-0.5);
//        }else if (opMode.gamepad1.right_stick_button){
//            shootersub.setVert(0.5);
//        }


        if (opMode.gamepad2.y) {
            servos.setVertTOP();
        }
//        if (opMode.gamepad1.b) { // Hold button to auto-aim
//
//            // Check if the camera actually sees the tag right now
//            if (cluster.isTagDetected()) {
//
//                // Get the horizontal offset (you must add this method to your Cluster class)
//                if (cluster.getDistance() > 5) {
//                    double newupTarget = cluster.getDistance() + 100;
//                    shootersub.uplock(newupTarget);
//                }
//
//                // Calculate the new absolute target: Current Position + Offset
//
//                // Command the PIDF to hold this new position
//
//            } else {
//                // Optional: If it loses sight of the tag, stay where it is,
//                // or return to a default center position (e.g., 0.0)
//                shootersub.uplock(100.0);
//            }
//        }
        if (opMode.gamepad2.dpad_up) {
            aimbot = true;
        }
        // 2. Unified Aimbot Loop using Alliance Filtering
        if (aimbot) {
            // Check specifically for tags matching our alliance
            if (cluster.isAllianceTagDetected(isRedAlliance)) {

                // A. Horizontal Turret Tracking
                double tagBearing = cluster.getAllianceBearing(isRedAlliance);
                if (Math.abs(tagBearing) > 0.0) {
                    double newTarget = shootersub.getcurrentPosition() - (tagBearing * 0.9);//0.7
                    shootersub.shootlock(newTarget);
                }

                // B. Distance-Based Hood Servo Adjustment
                double currentDist = cluster.getAllianceDistance(isRedAlliance);
                double entryAngle = cluster.getAllianceEntryAngle(isRedAlliance); // <-- ADD THIS LINE

                if (currentDist >= 100) {
                    servos.setVertBOTTOM();
                } else if (currentDist >= 90) {
                    servos.setVertBOTTOM();
                } else if (currentDist >= 80) {
                    servos.setVertMID();
                }

                if (opMode.gamepad2.right_trigger > 0) {
                    powersub.setPowerByDistance(currentDist, entryAngle); // Scales power based on Limelight distance
                } else {
                    powersub.gunidle(); // Turns off/idles the flywheels when you let go
                }
            }
        }

        // Unified Aimbot Loop: Handles both Turret Bearing and Hood Servo Distance Tracking
//        if (aimbot) {
//            if (cluster.isTagDetected()) {
//
//                // 1. Horizontal Turret Tracking
//                double tagBearing = cluster.getBearing();
//                if (Math.abs(tagBearing) > 1.5) {
//                    double newTarget = shootersub.getcurrentPosition() - (tagBearing * 0.7);
//                    shootersub.shootlock(newTarget);
//                }
//
//                // 2. Distance-Based Hood Servo Adjustment
//                double currentDist = cluster.getDistance();
//
//                if (currentDist >= 140) {
//                    servos.setVertBOTTOM();
//                } else if (currentDist >= 120) {
//                    servos.setVertX(0.0);
//                } else if (currentDist >= 100) {
//                    servos.setVertX(0.25);
//                } else if (currentDist >= 80) {
//                    servos.setVertX(0.5);
//                } else if (currentDist >= 60) {
//                    servos.setVertX(0.66);
//                } else if (currentDist >= 40) {
//                    servos.setVertX(0.75);
//                } else if (currentDist >= 20) {
//                    servos.setVertX(0.9);
//                } else if (currentDist > 0) {
//                    servos.setVertX(1.0);
//                }
//            }
//        }
//        if (opMode.gamepad2.dpad_up) {
//            aimbot = true;
//        }
//
//
//        if (aimbot == true) { // Hold button to auto-aim
//
//            // Check if the camera actually sees the tag right now
//            if (cluster.isTagDetected()) {
//
//                // Get the horizontal offset (you must add this method to your Cluster class)
//                double tagBearing = cluster.getBearing();
//                double distance = cluster.getDistance();
//                if (Math.abs(tagBearing) > 3 && distance > 20) {
//                    double newTarget = shootersub.getcurrentPosition() - (tagBearing);
//                    shootersub.shootlock(newTarget);
//                }
//
//                // Calculate the new absolute target: Current Position + Offset
//
//                // Command the PIDF to hold this new position
//
//            }
////                else {
////                    // Optional: If it loses sight of the tag, stay where it is,
////                    // or return to a default center position (e.g., 0.0)
////                    shootersub.shootlock(180.0);
////                }
//        }
//        if (aimbot == true) {
//            if (cluster.getDistance() > 140 && cluster.getDistance() < 160) {
//                servos.setVertBOTTOM();
//            }
//            if (cluster.getDistance() > 120 && cluster.getDistance() < 140) {
//                servos.setVertX(0);
//            }
//            if (cluster.getDistance() > 100 && cluster.getDistance() < 120) {
//                servos.setVertX(0.25);
//            }
//            if (cluster.getDistance() > 80 && cluster.getDistance() < 100) {
//                servos.setVertX(0.5);
//            }
//            if (cluster.getDistance() > 60 && cluster.getDistance() < 80) {
//                servos.setVertX(0.66);
//            }
//            if (cluster.getDistance() > 40 && cluster.getDistance() < 60) {//
//                servos.setVertX(0.75);
//            }
//            if (cluster.getDistance() > 20 && cluster.getDistance() < 40) {
//                servos.setVertX(0.9);
//            }
//            if (cluster.getDistance() > 0 && cluster.getDistance() < 20) {
//                servos.setVertX(1);
//            }
//        }


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
//            servosub.Swivelon();
//        }
//        else if (opMode.gamepad2.dpad_down){
//            servosub.Swiveloff();
//        }


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        //if (imc.hasMovedOnInit()){
        //  shootersub.update();

        //}
        shootersub.update();
        servos.update();
        powersub.update();
        buildtelemetry();
        sendDashboardPacket();

    }

    public Action Turn(double angle) {
        Scribe.getInstance().logData("here");
        return drive.actionBuilder(drive.localizer.getPose())
                .turnTo(Math.toRadians(angle + 15)).build();
    }

    public void lockit(double angle) {
        TelemetryPacket p = new TelemetryPacket();


//        Scribe.getInstance().logData(angle);
        if (angle != 0 && Math.abs(angle) > 20) {
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

