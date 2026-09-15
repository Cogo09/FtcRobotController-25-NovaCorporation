package org.firstinspires.ftc.teamcode.TELEOP;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;
import org.gentrifiedApps.gentrifiedAppsUtil.teleopTracker.TeleOpCopyRunner;
import org.gentrifiedApps.gentrifiedAppsUtil.teleopTracker.TeleOpTrackerOpMode;

public final class REPEATER {

    static String name = "Launch";
    static Driver driver = new Driver();


    static boolean isEnabled = true;

    private REPEATER() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, OpModeMeta.Flavor flavor) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup("Repeater")
                .setFlavor(flavor)
                .build();
    }
    //"frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor", DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!isEnabled) return;
        driver.setFrontLeftName("frontLeftMotor").setFrontRightName("frontRightMotor").setBackLeftName("backLeftMotor").setBackRightName("backRightMotor").reverseFrontLeft();
        manager.register(metaForClass(TeleOpCopyRunner.class, OpModeMeta.Flavor.AUTONOMOUS), new TeleOpCopyRunner(name, driver));
        manager.register(metaForClass(TeleOpTrackerOpMode.class, OpModeMeta.Flavor.TELEOP), new TeleOpTrackerOpMode(name, driver));
    }}

