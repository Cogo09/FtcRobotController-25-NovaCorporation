package org.firstinspires.ftc.teamcode.SUBS;

import static org.firstinspires.ftc.teamcode.UTILITIES.UTIL.setpose;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HARDWARES.SERVOUTIL;

import java.util.List;

public class SERVOSUB {
    private final Servo safetyswitch;



    public enum SafetyState {ON, OFF, IDLE}

    private SafetyState SafetyStateVar = SafetyState.IDLE;

    public void SafetyON() {
        SafetyStateVar = SafetyState.ON;
    }

    public void Safetyoff() {
        SafetyStateVar = SafetyState.OFF;
    }






    //this is where you put all enums and variables
    public SERVOSUB(HardwareMap hwMap) {
        safetyswitch = hwMap.get(Servo.class, "safetyswitch");

        safetyswitch.setDirection(Servo.Direction.FORWARD);


    }

    public void update() {
        // this is where you put your state machines and all power functions (call this in our main code)l

        switch (SafetyStateVar) {
            case ON:
                setpose(safetyswitch, SERVOUTIL.safetyon);
                break;
            case OFF:
                setpose(safetyswitch, SERVOUTIL.safetyoff);
                break;
            case IDLE:

                break;
        }



    }

    // this is where you put your update functions to switch between states
    public void telemetry(Telemetry telemetry) {
        // add telemetry data here

    }

    public Action ServoAction(SERVOSUB servosub, List<Runnable> funcs) {
        return new ServoAction(servosub,funcs);
    }

    class ServoAction implements Action {
        List<Runnable> funcs;
        private SERVOSUB servosub;

        public ServoAction(SERVOSUB servosub, List<Runnable> funcs) {
            this.funcs = funcs;
            this.servosub = servosub;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (Runnable func : funcs) {
                func.run();
            }
            servosub.update();// removes the need for the update to be run after simply updating a claw

            return false;
        }
    }
    public Action servoAction(List<Runnable> funcs){
        return new ServoAction(this, funcs);
    }
}