package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.UTILITIES.FlyUTIL;


@TeleOp(name = "flytune")
public class flytune extends LinearOpMode {
    public DcMotorEx shooterL;
    public DcMotorEx shooterR;

    public enum Velocity {High, Low}

    private Velocity VelocityStateVar = Velocity.High;

    double currenttargetvelo = FlyUTIL.highvelo;


    @Override
    public void runOpMode() {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FlyUTIL.p, 0, 0, FlyUTIL.f);
        shooterL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();
        while (opModeIsActive()) {
            if (VelocityStateVar == Velocity.High && gamepad2.crossWasPressed()) {
                VelocityStateVar = Velocity.Low;
                currenttargetvelo = FlyUTIL.lowvelo;
            } else if (VelocityStateVar == Velocity.Low && gamepad2.crossWasPressed()) {
                VelocityStateVar = Velocity.High;
                currenttargetvelo = FlyUTIL.highvelo;
            }

            PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(FlyUTIL.p, 0, 0, FlyUTIL.f);
            shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients2);
            shooterL.setVelocity(currenttargetvelo);
            shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients2);
            shooterR.setVelocity(currenttargetvelo);

            double currentVelo = shooterL.getVelocity();
            telemetry.addData("targetvelo", currenttargetvelo);
            telemetry.addData("currentVelo", currentVelo);
            telemetry.addData("current Velo", currentVelo);
            telemetry.update();
        }
    }
}
