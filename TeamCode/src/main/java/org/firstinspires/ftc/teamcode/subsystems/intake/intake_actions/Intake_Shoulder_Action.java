package org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Shoulder_Action {
    private Servo Intake_Shoulder;

    private static double INIT = 0.05;;
    private static double STOW = 0.05;
    private static double PICKUP_PREP = 0.5;
    private static double DROP = 1;
    private static double SAFE_MIN = 0;
    private static double SAFE_MAX = 1;
    private static double PARALLEL = 0.858;
    private static double HAND_OFF = 0.5;
    private static double AUTO_RIGHT_PICKUP = 0.5;
    private static double AUTO_LEFT_PICKUP = 0.5;

    public Intake_Shoulder_Action(HardwareMap hardwareMap) {
        this.Intake_Shoulder = hardwareMap.get(Servo.class, "Shoulder");
        this.Intake_Shoulder.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position){
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN: position;
        Intake_Shoulder.setPosition(position);
    }

    public double getCurrentPosition(){return Intake_Shoulder.getPosition();}

    public class IntakeShoulderStow implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(STOW);
                initialized = true;
            }
            return false;
        }

    }


    public class IntakeShoulderInit implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeShoulderPickUpPrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(PICKUP_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeShoulderAutoRightPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(AUTO_RIGHT_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeShoulderDrop implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(DROP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeShoulderParallel implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(PARALLEL);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeShoulderHandOff implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(HAND_OFF);
                initialized = true;
            }
            return false;
        }

    }
    public class IntakeShoulderAutoLeftPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Intake_Shoulder.setPosition(AUTO_LEFT_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public Action intakeShoulderInit(){return new IntakeShoulderInit();}
    public Action intakeShoulderStow(){return new IntakeShoulderStow();}
    public Action intakeShoulderPickUpPrep(){return new IntakeShoulderPickUpPrep();}
    public Action intakeShoulderAutoRightPickUp(){return new IntakeShoulderAutoRightPickUp();}
    public Action intakeShoulderDrop(){return new IntakeShoulderDrop();}
    public Action intakeShoulderParallel(){return new IntakeShoulderParallel();}
    public Action intakeShoulderHandOff(){return new IntakeShoulderHandOff();}
    public Action intakeShoulderAutoLeftPickUp(){return new IntakeShoulderAutoLeftPickUp();}
}