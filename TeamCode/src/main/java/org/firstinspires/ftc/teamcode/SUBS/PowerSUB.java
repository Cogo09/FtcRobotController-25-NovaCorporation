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
    private DcMotor intakeR;
    private DcMotor intakeL;
    private DcMotorEx shooterR;
    private DcMotorEx shooterL;

    public enum gunSTATE {ON, OFF, EXTRA, LITTLE, EXTRALITTLE, IDLE}

    //!help
    private PowerSUB.gunSTATE gunStateVar = PowerSUB.gunSTATE.IDLE;

    public void power() {
        gunStateVar = gunSTATE.ON;
    }

    public void littlepower() {
        gunStateVar = gunSTATE.LITTLE;
    }
    public void extralittle(){gunStateVar = gunSTATE.EXTRALITTLE;}

    public void gunoff() {
        gunStateVar = gunSTATE.OFF;
    }

    public void extrapower() {
        gunStateVar = gunSTATE.EXTRA;
    }

    public void gunidle() {
        gunStateVar = gunSTATE.IDLE;
    }

    public enum intakeSTATE {ON, OFF,ONSLOW, ONL, REVERSE, REVERSESLIGHT, IDLE}

    private PowerSUB.intakeSTATE intakeStateVar = PowerSUB.intakeSTATE.IDLE;

    public void intakeon() {
        intakeStateVar = intakeSTATE.ON;
    }

    public void intakeonl() {
        intakeStateVar = intakeSTATE.ONL;
    }
    //p

    public void intakeoff() {
        intakeStateVar = intakeSTATE.OFF;
    }

    public void intakereverse() {
        intakeStateVar = intakeSTATE.REVERSE;
    }

    public void intakereverseS() {
        intakeStateVar = intakeSTATE.REVERSESLIGHT;
    }
    public void intakeonslow(){intakeStateVar = intakeSTATE.ONSLOW;}


    //this is where you put all enums and variables
    public PowerSUB(HardwareMap hwMap) {
        shooterL = hwMap.get(DcMotorEx.class, "shooterL");
        shooterR = hwMap.get(DcMotorEx.class, "shooterR");
        intakeR = hwMap.get(DcMotor.class, "intakeR");
        intakeL = hwMap.get(DcMotor.class, "intakeL");
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FlyUTIL.p, 0, 0, FlyUTIL.f);
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)
        switch (intakeStateVar) {
            case ONL:
                intakeL.setPower(0.9);

                break;
            case ON:
                intakeL.setPower(0.75);
                intakeR.setPower(0.75);
                break;

            case OFF:
                intakeR.setPower(0);
                intakeL.setPower(0);
                break;
            case REVERSE:
                intakeR.setPower(-1);
                intakeL.setPower(-1);
                break;
            case ONSLOW:
                intakeR.setPower(0.5);
                intakeL.setPower(0.5);
            case REVERSESLIGHT:
                intakeR.setPower(-1);
                intakeL.setPower(-1);
                break;
            case IDLE:

                break;
        }

        switch (gunStateVar) {
            case ON:
                shooterL.setVelocity(0.465 * FlyUTIL.highvelo);

                shooterR.setVelocity(0.465 * FlyUTIL.highvelo);
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
                shooterL.setVelocity(0.42*FlyUTIL.highvelo);
                shooterR.setVelocity(0.42*FlyUTIL.highvelo);
                break;

            case EXTRA:
                shooterR.setVelocity(0.56 * FlyUTIL.highvelo);

                shooterL.setVelocity(0.56 * FlyUTIL.highvelo);
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
        }


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