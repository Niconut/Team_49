package org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Elbow_Action {
    private Servo IntakeElbow;

    private  double INIT = 0.625;
    private  double STOW = 0.625;
    private  double DROP = 0.65;
    private  double PICKUP_PREP = 0.68; //0.555;
    private  double PICKUP = 0.7;
    private double PICKUP_DONE = 0.65;
    private double HAND_OFF = 0.515;


    public Intake_Elbow_Action(HardwareMap hardwareMap) {
        this.IntakeElbow = hardwareMap.get(Servo.class, "Elbow");
        this.IntakeElbow.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){
        IntakeElbow.setPosition(position);}

    public double getCurrentPosition(){return IntakeElbow.getPosition();}

    public class IntakeElbowInit implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeElbowStow implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(STOW);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeElbowDrop implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(DROP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeElbowPickUpPrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(PICKUP_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeElbowPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeElbowPickUpDone implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(PICKUP_DONE);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeElbowHandOff implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeElbow.setPosition(HAND_OFF);
                initialized = true;
            }
            return false;
        }

    }

    public Action intakeElbowInit(){return new IntakeElbowInit();}
    public Action intakeElbowStow(){return new IntakeElbowStow();}
    public Action intakeElbowDrop(){return new IntakeElbowDrop();}
    public Action intakeElbowPickUpPrep(){return new IntakeElbowPickUpPrep();}
    public Action intakeElbowPickUp(){return new IntakeElbowPickUp();}
    public Action intakeElbowPickUpDone(){return new IntakeElbowPickUpDone();}
    public Action intakeElbowHandOff(){return new IntakeElbowHandOff();}
}