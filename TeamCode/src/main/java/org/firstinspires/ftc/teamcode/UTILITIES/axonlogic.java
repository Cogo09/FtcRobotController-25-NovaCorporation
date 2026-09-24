package org.firstinspires.ftc.teamcode.UTILITIES;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.gentrifiedApps.gentrifiedAppsUtil.classes.analogEncoder.AnalogEncoder;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.AxonServo;
import org.gentrifiedApps.gentrifiedAppsUtil.hardware.servo.ServoPlus;
import org.firstinspires.ftc.teamcode.UTILITIES.MathFunctions;

enum AxonAlgorithm {
    REGULAR,
    REVERSED;

    public AxonAlgorithm reversed() {
        return (this == REGULAR) ? REVERSED : REGULAR;
    }
}

public class axonlogic {
    private final String name;
    private AnalogEncoder encoder;
    private CRServo servo;
    private AxonAlgorithm algorithm = AxonAlgorithm.REGULAR;


    public axonlogic(HardwareMap hw, String name) {
        this.name = name;
        this.encoder = initAEncoder(hw);
        this.servo = hw.get(CRServo.class, name);

    }

    public axonlogic setAlgo(AxonAlgorithm algorithm) {
        this.algorithm = algorithm;
        return this;
    }

//    public void checkError() {
//        double errorTolerance = 20;
//        if (errorTolerance > getError()) {
//            setAlgo(this.algorithm.reversed());
//        }
//    }
    //

    private AnalogEncoder initAEncoder(HardwareMap hw) {
        return AnalogEncoder.axon(hw, name + "Encoder");
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData(name + " aEncoder", "%.1f", this.getEncoderPosition());
    }

    public void setPosition(double degree) {
        servo.setPower(degree);
    } // rename this to its actual usage

//    public void setPositionCheck(double degree) {
//        servo.setPower(degree);
//        checkError();
//    }

    public double getEncoderPosition() {
        switch (algorithm) {
            case REVERSED:
                return getEncoderPositionReversed();
            case REGULAR:
            default:
                return getEncoderPositionRegular();
        }
    }

    public double getEncoderPositionRegular() {
        return (double) encoder.getCurrentPosition();
    }

    public double getEncoderPositionReversed() {
        return (((encoder.getVoltage() - 3.3) / 3.3) * 360) * -1;
    }

}