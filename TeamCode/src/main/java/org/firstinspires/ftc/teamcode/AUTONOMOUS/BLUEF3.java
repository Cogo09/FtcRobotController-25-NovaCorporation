package org.firstinspires.ftc.teamcode.AUTONOMOUS;

import static org.firstinspires.ftc.teamcode.HARDWARES.HARDWARECONFIG.currentpose;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HARDWARES.AUTOHARDWARE;


@Autonomous
public class BLUEF3 extends LinearOpMode {
    AUTOHARDWARE robot = null;
    //
    @Override//
    public void runOpMode() throws InterruptedException {
        robot = new AUTOHARDWARE(this, hardwareMap, new Pose2d(-56,60,Math.toRadians(270)));
        waitForStart();
        if (opModeIsActive()){
            robot.BLUEF3();

            blackboard.put(currentpose,robot.drive.localizer.getPose());
        }
    }
}

