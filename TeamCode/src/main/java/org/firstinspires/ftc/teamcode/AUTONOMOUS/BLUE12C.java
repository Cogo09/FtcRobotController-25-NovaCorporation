package org.firstinspires.ftc.teamcode.AUTONOMOUS;

import static org.firstinspires.ftc.teamcode.HARDWARES.HARDWARECONFIG.currentpose;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HARDWARES.AUTOHARDWARE;


@Autonomous
public class BLUE12C extends LinearOpMode {
    AUTOHARDWARE robot = null;
    ///
    ///
    ///
    ///
    ///
    ///
    ///THIS FILE IS ONLY FOR TESTING AUTONOMOUS OUT NO AUTO RUNS ARE TO BE STORED HERE!!!!!!!!!
    ///
    ///
    ///
    ///
    ///
    @Override//
    public void runOpMode() throws InterruptedException {
        robot = new AUTOHARDWARE(this, hardwareMap, new Pose2d(-52,-50,Math.toRadians(220)));
        waitForStart();
        if (opModeIsActive()){
            robot.BLUE12C();

            blackboard.put(currentpose,robot.drive.localizer.getPose());
        }
    }
}

