package org.firstinspires.ftc.teamcode.HARDWARES;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HARDWARES.SERVOUTIL;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

public class Turretaimbot {
    private CRServo swivelL;
    private CRServo swivelR;

    private double kP = 0.000000001;
    private double kD = 0.0000000;
    private double goalx = 0;
    private double lastError = 0;
    private double angleTolerage = 0.2;
    private final double MAX_POWER = 0.5;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwmap) {
        swivelR = hwmap.get(CRServo.class, "swivelR");
        swivelL = hwmap.get(CRServo.class, "swivelL");
    }
    public void setkP(double newkP) {
        kP = newkP;
    }
    public double getkP() {
        return kP;
    }
    public void setkD(double newkD) {
        kD = newkD;
    }
    public double getkD() {
        return kD;
    }
    public void resetTimer() {
        timer.reset();
    }
    public void update(AprilTagClusterDetection detection) {
        double deltaTime = timer.seconds();
        timer.reset();

        if(detection == null){
            swivelL.setPower(0);
            swivelR.setPower(0);
            lastError = 0;
            return;
        }


        double error = goalx - detection.ftcPose.bearing;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }
        if (Math.abs(error)< angleTolerage){
            power = 0;

        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);

        }
        swivelR.setPower(power);
        swivelL.setPower(power);
        lastError = error;

    }
}
