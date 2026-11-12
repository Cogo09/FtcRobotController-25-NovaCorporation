package org.firstinspires.ftc.teamcode.SUBS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UTILITIES.FlyUTIL;

import java.util.List;

public class PowerSUB {
    private DcMotor intakeReg;
    private DcMotor intakeLock;
    private DcMotorEx shooterR;
    private DcMotorEx shooterL;
    // 1. Add these two fields near the top of PowerSUB:
    private double targetDistance = 0.0;
    private double targetEntryAngle = 0.0;

    // 2. Update setPowerByDistance to accept both distance and entry angle:



    public enum gunSTATE {ON, OFF, EXTRA, LITTLE, EXTRALITTLE,DISTANCE_BASED, IDLE}

    //!help
    private PowerSUB.gunSTATE gunStateVar = PowerSUB.gunSTATE.IDLE;

    public void power() {
        gunStateVar = gunSTATE.ON;
    }

    public void setPowerByDistance(double distanceInches, double entryAngleDegrees) {
        gunStateVar = gunSTATE.DISTANCE_BASED;
        this.targetDistance = distanceInches;
        this.targetEntryAngle = Math.abs(entryAngleDegrees);
    }

    public void littlepower() {
        gunStateVar = gunSTATE.LITTLE;
    }

    public void extralittle() {
        gunStateVar = gunSTATE.EXTRALITTLE;
    }

    public void gunoff() {
        gunStateVar = gunSTATE.OFF;
    }

    public void extrapower() {
        gunStateVar = gunSTATE.EXTRA;
    }

    public void gunidle() {
        gunStateVar = gunSTATE.IDLE;
    }

    public enum intakeSTATE {FRONTON, OFF, ALLON, REVERSE, IDLE}

    private PowerSUB.intakeSTATE intakeStateVar = PowerSUB.intakeSTATE.IDLE;

    public void fronton() {
        intakeStateVar = intakeSTATE.FRONTON;
    }

    public void allon() {
        intakeStateVar = intakeSTATE.ALLON;
    }
    //p

    public void intakeoff() {
        intakeStateVar = intakeSTATE.OFF;
    }

    public void intakeidle() {
        intakeStateVar = intakeSTATE.IDLE;
    }

    public void intakereverse() {
        intakeStateVar = intakeSTATE.REVERSE;
    }


    //this is where you put all enums and variables
    public PowerSUB(HardwareMap hwMap) {
        shooterL = hwMap.get(DcMotorEx.class, "shooterL");
        shooterR = hwMap.get(DcMotorEx.class, "shooterR");
        intakeReg = hwMap.get(DcMotor.class, "intakeReg");
        intakeLock = hwMap.get(DcMotor.class, "intakeLock");
        intakeReg.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLock.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLock.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeReg.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FlyUTIL.p, 0, 0, FlyUTIL.f);
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)
        switch (intakeStateVar) {
            case ALLON:
                intakeLock.setPower(1);
                intakeReg.setPower(1);


                break;
            case FRONTON:
                intakeReg.setPower(1);
                break;

            case OFF:
                intakeReg.setPower(0);
                intakeLock.setPower(0);
                break;
            case REVERSE:
                intakeReg.setPower(-1);
                intakeLock.setPower(-1);
                break;
            case IDLE:
                intakeLock.setPower(0);
                intakeReg.setPower(0);

                break;
        }

        switch (gunStateVar) {
            case ON:
                shooterL.setVelocity(0.5 * FlyUTIL.highvelo);

                shooterR.setVelocity(0.5 * FlyUTIL.highvelo);
                break;
            case DISTANCE_BASED:
                // Map distance to a velocity multiplier factor
                // Adjust min/max distance and multipliers to match your robot's physical shooting curve!
                double minDistance = 85.0;  // Inches (closest range)
                double maxDistance = 112.0; // Inches (farthest range)
                double minFactor = 0.60;    // 35% velocity at close range
                double maxFactor = 0.75;    // 85% velocity at max range

                // Clamp distance within safe bounds
                double clampedDist = Math.max(minDistance, Math.min(targetDistance, maxDistance));
                double distanceFactor = minFactor + (clampedDist - minDistance) / (maxDistance - minDistance) * (maxFactor - minFactor);
                double maxExpectedAngle = 45.0; // Degrees
                double maxAngleBoost = 0.12;    // Tune this up if angled shots still fall short, or down if they overshoot
                // Calculate linear scale factor
                double scaleFactor = minFactor + (clampedDist - minDistance) / (maxDistance - minDistance) * (maxFactor - minFactor);
                double clampedAngle = Math.min(targetEntryAngle, maxExpectedAngle);
                double angleBoost = (clampedAngle / maxExpectedAngle) * maxAngleBoost;

                double totalFactor = Math.min(1.0, distanceFactor + angleBoost);

                double targetVelo = scaleFactor * FlyUTIL.highvelo;
                shooterL.setVelocity(targetVelo);
                shooterR.setVelocity(targetVelo);
                break;
            case OFF:
                shooterL.setVelocity(0.0);

                shooterR.setVelocity(0.0);
                break;
            case LITTLE:
                shooterL.setVelocity(0.44 * FlyUTIL.highvelo);
                shooterR.setVelocity(0.44 * FlyUTIL.highvelo);
                break;
            case EXTRALITTLE:
                shooterL.setVelocity(0.42 * FlyUTIL.highvelo);
                shooterR.setVelocity(0.42 * FlyUTIL.highvelo);
                break;//

            case EXTRA:
                shooterR.setVelocity(0.68 * FlyUTIL.highvelo);

                shooterL.setVelocity(0.68 * FlyUTIL.highvelo);
                break;
            case IDLE:
                shooterR.setVelocity(0.0 * FlyUTIL.highvelo);

                shooterL.setVelocity(0.0 * FlyUTIL.highvelo);
                break;
        }
    }

    // this is where you put your update functions to switch between states

    public void telemetry(Telemetry telemetry) {
        double currentVeloL = shooterR.getVelocity();
        double currentVeloR = shooterL.getVelocity();
        telemetry.addData("RVELO", currentVeloR);
        telemetry.addData("LVELO", currentVeloL);

    }
    // add telemetry data here


    class MotorAction implements Action {
        List<Runnable> funcs;
        private PowerSUB powersub;

        public MotorAction(PowerSUB powersub, List<Runnable> funcs) {
            this.funcs = funcs;
            this.powersub = powersub;
        }//m


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            powersub.update();// removes the need for the update to be run after simply updating a claw

            return false;
        }

    }

    public Action gunAction(List<Runnable> funcs) {
        return new MotorAction(this, funcs);
    }
}