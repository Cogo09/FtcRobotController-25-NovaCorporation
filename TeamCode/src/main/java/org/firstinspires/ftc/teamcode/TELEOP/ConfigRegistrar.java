package org.firstinspires.ftc.teamcode.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.drive.drift.DriftTunerOpMode;
import org.gentrifiedApps.gentrifiedAppsUtil.config.ConfigCreator;
import org.gentrifiedApps.gentrifiedAppsUtil.config.ConfigMaker;
import org.gentrifiedApps.gentrifiedAppsUtil.heatseeker.Driver;

public final class ConfigRegistrar {

    static ConfigMaker config = new ConfigMaker("PROPERTY OF NOVA CORP")
            .addModule(ConfigMaker.ModuleType.EXPANSION_HUB, "Expansion Hub 1", 1)
            .addCamera("Webcam 1","UC684")
            .addMotor("frontLeftMotor", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.goBILDA5201SeriesMotor, 0)
            .addMotor("backLeftMotor", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.goBILDA5201SeriesMotor, 1)
            .addMotor("frontRightMotor", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.goBILDA5201SeriesMotor, 2)
            .addMotor("backRightMotor", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.MotorType.goBILDA5201SeriesMotor, 3)
//            .addMotor("intakeL", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.RevRoboticsCoreHexMotor, 2)
//            .addMotor("intakeR", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.RevRoboticsCoreHexMotor, 3)
//            .addMotor("shooterL", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.goBILDA5201SeriesMotor, 1)
//            .addMotor("shooterR", ConfigMaker.ModuleType.EXPANSION_HUB, ConfigMaker.MotorType.goBILDA5201SeriesMotor, 0)
            .addDevice("turretleft", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.Servo, 0)
            .addDevice("turretvert", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.Servo, 1)
            .addDevice("turretright", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.Servo, 2)
            .addDevice("turretleftEncoder", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.AnalogInput,0) //! good job here (i have no clue if the ports are correct)
            .addDevice("turretrightEncoder", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.AnalogInput,1)
            .addDevice("turretvertEncoder", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.AnalogInput,2)
            .addDevice("imu", ConfigMaker.ModuleType.CONTROL_HUB, ConfigMaker.DeviceType.REV_INTERNAL_BHI260_IMU, 0);



    static boolean isEnabled = true;

    private ConfigRegistrar() {
    }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup("Config")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    //!hihihimm
    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (!isEnabled) return;
        Driver driver = new Driver();
        driver.setFrontLeftName("frontLeftMotor").setFrontRightName("frontRightMotor").setBackLeftName("backLeftMotor").setBackRightName("backRightMotor").reverseFrontLeft();
        manager.register(metaForClass(ConfigCreator.class), new ConfigCreator(config));
        manager.register(metaForClass(DriftTunerOpMode.class), new DriftTunerOpMode(driver, 3));

    }
}
    