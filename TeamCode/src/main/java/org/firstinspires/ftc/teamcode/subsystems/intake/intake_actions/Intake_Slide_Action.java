package org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Slide_Action {
    private Servo Intake_Slide;

    private static double INIT = 0.52; //0.425;
    private static double STOW = 0.52;
    private static double PICKUP_PREP = 0.495;
    private static double SAFE_MIN = 0.13;
    private static double HAND_OFF_PREP = 0.51;
    private static double HAND_OFF = 0.51;
    private static double MID = 0.5;
    private static double SAFE_MAX = 0.52;
    private double AUTO_LEFT_PICKUP = 0.425;

    public Intake_Slide_Action(HardwareMap hardwareMap) {
        this.Intake_Slide = hardwareMap.get(Servo.class, "SlideLeft");
        this.Intake_Slide.setDirection(Servo.Direction.FORWARD);
    }

    public double getCurrentPosition(){return Intake_Slide.getPosition();}

    public void setPosition(double position){
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN: position;
        Intake_Slide.setPosition(position);
    }


    public class IntakeSlideInit implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeSlideStow implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(STOW);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeSlidePickUpPrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(PICKUP_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeSlidePickUpMid implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(MID);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeSlideHandOffPrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(HAND_OFF_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeSlideHandOff implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(HAND_OFF);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeSlideAutoLeftPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Slide.setPosition(AUTO_LEFT_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public Action intakeSlideInit(){return new IntakeSlideInit();}
    public Action intakeSlideStow(){return new IntakeSlideStow();}
    public Action intakeSlidePickUpPrep(){return new IntakeSlidePickUpPrep();}
    public Action intakeSlidePickUpMid(){return new IntakeSlidePickUpMid();}
    public Action intakeSlideHandOffPrep(){return new IntakeSlideHandOffPrep();}
    public Action intakeSlideHandOff(){return new IntakeSlideHandOff();}
    public Action intakeSlideAutoLeftPickUp(){return new IntakeSlideAutoLeftPickUp();}
}