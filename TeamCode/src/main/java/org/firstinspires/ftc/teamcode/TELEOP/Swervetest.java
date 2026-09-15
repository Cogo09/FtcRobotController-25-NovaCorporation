package org.firstinspires.ftc.teamcode.TELEOP;

import static org.firstinspires.ftc.teamcode.UTILITIES.Swerveutil.swervee;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class Swervetest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveMod module = new SwerveMod("motor", hardwareMap, "servo", "enc", 0,swervee);
        waitForStart();

        while (opModeIsActive()){
            module.setpids(swervee);
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double power = -gamepad1.right_stick_y;
            module.turnservo(x,y, telemetry);
            module.setpower(power);
            module.writer(telemetry);
            telemetry.update();
        }
    }
}
