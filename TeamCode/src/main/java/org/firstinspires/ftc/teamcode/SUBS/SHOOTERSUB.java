package org.firstinspires.ftc.teamcode.SUBS;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.d;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.p;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.i;
import static org.firstinspires.ftc.teamcode.UTILITIES.AXONutil.f;
import static org.firstinspires.ftc.teamcode.UTILITIES.TURRETUTIL.ddd;
import static org.firstinspires.ftc.teamcode.UTILITIES.VertAXONutil.dd;
import static org.firstinspires.ftc.teamcode.UTILITIES.VertAXONutil.pp;
import static org.firstinspires.ftc.teamcode.UTILITIES.VertAXONutil.ii;
import static org.firstinspires.ftc.teamcode.UTILITIES.VertAXONutil.ff;
import static org.firstinspires.ftc.teamcode.UTILITIES.TURRETUTIL.ppp;
import static org.firstinspires.ftc.teamcode.UTILITIES.TURRETUTIL.iii;
import static org.firstinspires.ftc.teamcode.UTILITIES.TURRETUTIL.ddd;
import static org.firstinspires.ftc.teamcode.UTILITIES.TURRETUTIL.fff;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AXONS.vertPIDFController;
import org.firstinspires.ftc.teamcode.UTILITIES.axonlogic;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AXONS.PIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.UTILITIES.axonlogic;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.AxonServo;

import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.AxonServo;

import java.util.List;

public class SHOOTERSUB {
    private axonlogic turretleft;
    private axonlogic turretright;
    private Servo turretvert;
    private DcMotorEx turretcontrol;

    //MOTORS HERE LATER

    private boolean isHoldingPosition = false;
    private double targetPosition = 0;
    private boolean isHoldingUpPosition = false;
    private double targetUpPosition = 0;

    PIDFController pidfController = new PIDFController(p, i ,d, f );
    PIDController vertpidfcontroller = new PIDController(pp, ii ,dd );
    PIDController turretcontroller = new PIDController (ppp, iii ,ddd);
    public void init(HardwareMap hwmap){
        turretleft = new axonlogic(hwmap, "turretleft");
        turretright = new axonlogic(hwmap, "turretright");
        //turretvert = new Servo(hwmap, "turretvert");
        vertpidfcontroller.reset();
        //! 270 max down vertpidf
    }





    public void shootlock(double target){ //! cluster.range should get passed in as target
        this.targetPosition = target;
        shootunlock();
    }
    public void shootunlock(){
        isHoldingPosition = true;
    }
    public void uplock(double uptarget){
        this.targetUpPosition = uptarget;
        upunlock();

    }
    void upunlock(){
        isHoldingUpPosition = true;
    }
    public void lockshoot(){
        isHoldingPosition = false;
    }
    void lockup(){
        isHoldingUpPosition = false;
    }
    public double getcurrentPosition(){
        return turretleft.getEncoderPosition();
    }
//    public double transcription(){
//        return turretvert.getEncoderPositionRegular();
//    }

//    public double getcurrentVertPosition(){
//        return turretvert.getEncoderPosition();
//    }

    public double getavg(){
        return ((turretleft.getEncoderPosition()) + (turretright.getEncoderPosition())/2.0);
    }

//    public void setVert(double p){
//        turretvert.setpower(p);
//    }


    public void update(){
        pidfController.setPIDF(p,i,d,f);
        vertpidfcontroller.setPIDF(pp,ii,dd,ff);
        if (isHoldingPosition == true) {
            // Calculate power using your PIDF controller
            // (Make sure the argument order matches your PIDFController class: target, current)
            double val = pidfController.calculate(targetPosition, turretleft.getEncoderPositionRegular());
            if (Double.isNaN(val)){
                val = 0.0;
            }

            val = Math.max(-0.1, Math.min(0.1, val));


            // Send power to the left turret servo



            // Send power to the left turret servo
            turretleft.setpower(val);

            // Invert the power for the right servo so they work together instead of fighting
            turretright.setpower(val);




        }
        if (isHoldingPosition == true){
            double targetpose = pidfController.calculate(targetPosition, turretcontroller.getPositionError());
            if (Double.isNaN(targetpose)){
                targetpose = 0.0;
            }
            targetpose = Math.max(-0.1, Math.min(0.1, targetpose));
            turretcontrol.setPower(targetpose);
        }
//        if (isHoldingUpPosition == true) {
//            // Calculate power using your PIDF controller
//            // (Make sure the argument order matches your PIDFController class: target, current)
//            double vert = vertpidfcontroller.calculate(targetUpPosition, turretvert.getEncoderPositionReversed());
//            if (Double.isNaN(vert)){
//                vert = 0.0;
//            }
////            vert = Math.clamp(vert,-0.3,0.3);
//
//            // Send power to the left turret servo
//            turretvert.setpower(vert);
//        }

    }
    public void telemetry(Telemetry telemetry){
        telemetry.addData("turretleft", turretleft.getEncoderPositionRegular());
        telemetry.addData("turretright", turretright.getEncoderPositionRegular());
        telemetry.addData("turretmain", turretcontrol.getCurrentPosition());
//        telemetry.addData("turretvert", turretvert.getEncoderPositionRegular());
//        telemetry.addData("turretvert", turretvert.getEncoderPositionReversed());
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("target",turretleft.getpower());
//        telemetry.addData("target", pidfController.calculate(targetPosition, getavg()));
        telemetry.addData("targetUpPosition", targetUpPosition);
//        telemetry.addData("targetUp",turretvert.getpower());
//        telemetry.addData("targetUp", vertpidfcontroller.calculate(targetUpPosition, turretvert.getEncoderPosition()));

    }

}
