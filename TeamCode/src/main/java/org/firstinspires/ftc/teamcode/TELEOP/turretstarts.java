package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HARDWARES.Turretaimbot;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@TeleOp
public class turretstarts extends OpMode {
    private AprilTagClusterDetection aprilTagClusterDetection = new AprilTagClusterDetection();
    private Turretaimbot turret = new Turretaimbot();

    @Override
    public void init() {
        aprilTagClusterDetection.
        turret.init(hardwareMap);

    }

    public void start() {

    }

    @Override
    public void loop() {

    }
}
