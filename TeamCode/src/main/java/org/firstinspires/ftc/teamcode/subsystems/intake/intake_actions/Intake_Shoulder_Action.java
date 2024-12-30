package org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Shoulder_Action {
    private Servo Intake_Shoulder;

    private static double INIT = 0.16;
    private static double STOW = 0.16;
    private static double PICKUP_PREP = 0.69;
    private static double AutoRightPickUp = 0.5;
    private static double DROP = 0.25;
    private double SAFE_MIN = 0.15;
    private double SAFE_MAX = 0.8;

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
                Intake_Shoulder.setPosition(AutoRightPickUp);
                initialized = true;
            }
            return false;
        }

    }

    public Action intakeShoulderStow(){return new IntakeShoulderStow();}
    public Action intakeShoulderPickUpPrep(){return new IntakeShoulderPickUpPrep();}
    public Action intakeShoulderAutoRightPickUp(){return new IntakeShoulderAutoRightPickUp();}
}