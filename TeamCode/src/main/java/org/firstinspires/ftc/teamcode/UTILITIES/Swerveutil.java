package org.firstinspires.ftc.teamcode.UTILITIES;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class Swerveutil {
    public static PIDCoefficients swervee = new PIDCoefficients(0.00, 0, 0.000);//0.005 works

}
