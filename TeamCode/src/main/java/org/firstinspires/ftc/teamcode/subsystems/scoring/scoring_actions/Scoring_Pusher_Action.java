package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Pusher_Action {
    private Servo Pusher;
    private double START = 0.21;
    private double EXTEND = 0.35;
    private double RETRACT = 0.21;
    public Scoring_Pusher_Action(HardwareMap hardwareMap) {
        Pusher = hardwareMap.get(Servo.class, "Pusher");
    }

    public void setPosition(double position) {
        Pusher.setPosition(position);
    }
    public class StartPusher implements Action{
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Pusher.setPosition(START);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }
    public class RetractPusher implements Action{
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Pusher.setPosition(RETRACT);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class ExtendPusher implements Action{
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Pusher.setPosition(EXTEND);
                initialized = true;
                /*double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }
    public Action StartPusher(){return new StartPusher();}
    public Action RetractPusher(){return new RetractPusher();}
    public Action ExtendPusher(){return new ExtendPusher();}

}
