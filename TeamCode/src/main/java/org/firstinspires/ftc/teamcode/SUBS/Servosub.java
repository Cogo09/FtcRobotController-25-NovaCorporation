package org.firstinspires.ftc.teamcode.SUBS;


import static org.firstinspires.ftc.teamcode.UTILITIES.UTIL.setpose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UTILITIES.SERVOUTIL;

import java.util.List;

public class Servosub {
    private final Servo ramp;
    private final Servo ramp2;


    public enum RampState {UP, DOWN, IDLE}

    private RampState RampStateVar = RampState.IDLE;

    public void setRampUP() {
        RampStateVar = RampState.UP;
    }

    public void setRampDOWN() {
        RampStateVar = RampState.DOWN;
    }

    public void setRampIDLE() {
        RampStateVar = RampState.IDLE;
    }



    //this is where you put all enums and variables
    public Servosub(HardwareMap hwMap) {
        ramp = hwMap.get(Servo.class, "rampservo");
        ramp2 = hwMap.get(Servo.class, "rampservo2");


    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)


        switch (RampStateVar) {
            case UP:
                setpose(ramp, SERVOUTIL.rampup);//actually close increase to close more
                setpose(ramp2, SERVOUTIL.rampup);

                break;
            case DOWN:
                setpose(ramp, SERVOUTIL.rampdown);//actually open decrease to open more
                setpose(ramp2, SERVOUTIL.rampdown);
                break;
            case IDLE:

                break;
        }
//        switch (SwitchStateVar) {
//            case PRIME:
//                setpose(switchservo, ServoUtil.switchprime);
//                setpose(switchservo1, ServoUtil.switchprime1);
//                break;
//            case LOAD:
//                setpose(switchservo, ServoUtil.switchload);
//                setpose(switchservo1, ServoUtil.switchload1);
//                break;
//
//        }



    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        // add telemetry data here

    }


    public Action rampAction(Servosub ServoSub, List<Runnable> funcs) {
        return new RampAction(ServoSub,funcs);
    }

    class RampAction implements Action {
        List<Runnable> funcs;
        private Servosub ServoSub;

        public RampAction(Servosub ServoSub, List<Runnable> funcs) {
            this.funcs = funcs;
            this.ServoSub = ServoSub;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            ServoSub.update();// removes the need for the update to be run after simply updating a

            return false;
        }
    }
}