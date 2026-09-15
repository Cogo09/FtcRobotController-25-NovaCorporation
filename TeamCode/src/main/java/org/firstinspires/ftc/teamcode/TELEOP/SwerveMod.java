package org.firstinspires.ftc.teamcode.TELEOP;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveMod {
    private DcMotor motor;
    private CRServo twisty;
    private AnalogInput encoder;
    private double mult = 0.0028;
    private double offset = 0;

    private  PIDController controller = new PIDController(0,0,0);

    PIDCoefficients pids = new PIDCoefficients(0,0,0);



    public SwerveMod(String motorname, HardwareMap hwmap, String servoname, String enc, double _offset, PIDCoefficients coefficients) {
        motor = hwmap.dcMotor.get(motorname);
        twisty = hwmap.crservo.get(servoname);
        encoder = hwmap.analogInput.get(enc);
        this.offset = _offset;
        setpids(coefficients);

    }

    public void setpids (PIDCoefficients _pids){
        pids = _pids;
        controller.setPID(pids.p,pids.i,pids.d);
    }

    private double getencangle() {
        double Voltage = encoder.getVoltage();
        double currentposition = (Voltage / 3.3) * 360;

        return currentposition;
    }

    private void turnservo(double angle, Telemetry telemetry) {
        angle -= offset;

        if (angle < 0) {
            angle += 180;
        }
        telemetry.addData("turnangle",angle);
        double power = controller.calculate(getencangle(), angle);
        twisty.setPower(power);
    }

    public void turnservo(double x, double y,Telemetry telemetry) {
        double angle = Math.toDegrees(Math.atan2(y,x));
        turnservo(angle, telemetry);
    }

    public void writer(Telemetry telemetry){
        telemetry.addData("angle",getencangle());
    }

    public void setpower(double power) {
        motor.setPower(power);

    }
}
