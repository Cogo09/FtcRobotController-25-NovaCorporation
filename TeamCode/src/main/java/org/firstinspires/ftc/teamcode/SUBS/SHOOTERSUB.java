package org.firstinspires.ftc.teamcode.SUBS;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.d;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.p;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.i;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.f;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.UTILITIES.axonlogic;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AXONS.PIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.UTILITIES.axonlogic;
//import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.AxonServo;

import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.AxonServo;
import org.opencv.core.Range;

import java.util.List;

public class SHOOTERSUB {
    private AxonServo turretleft;
    private AxonServo turretright;
    private AxonServo turretvert;
    Cluster cluster = new Cluster();
    //MOTORS HERE LATER
    public void init(HardwareMap hwmap){
        turretleft = new AxonServo(hwmap, "turretleft");
        turretright = new AxonServo(hwmap, "turretright");
        turretvert = new AxonServo(hwmap, "turretvert");
    }


    PIDFController pidfController = new PIDFController(p, i ,d, f);
    public double val = 0;


    public void shootlock(double target){
        val = pidfController.calculate(turretleft.getEncoderPosition(), target);
        turretleft.setPosition(val);
        turretright.setPosition(val);
    }


    public void update(){


    }
    public void telemetry(Telemetry telemetry){
        telemetry.addData("turretleft", turretleft.getEncoderPositionRegular());
        telemetry.addData("turretright", turretright.getEncoderPosition());
        telemetry.addData("turretvert", turretvert.getEncoderPosition());
        telemetry.addData("val",val);

    }

}
