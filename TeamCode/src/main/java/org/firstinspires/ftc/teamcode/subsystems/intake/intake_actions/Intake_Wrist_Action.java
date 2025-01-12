package org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Wrist_Action {
    private Servo Wrist;

    private double INIT = 0.5;
    private double DROP = 0.5;
    private double PICKUP_PREP = 0.5;
    private double STOW = 0.5;
    private double AUTO_RIGHT_PICKUP = 0.65;
    private double AUTO_LEFT_PICKUP = 0.39;
    private double SAFE_MAX = 0.75;
    private double SAFE_MIN = 0.25;


    public Intake_Wrist_Action(HardwareMap hardwareMap) {
        this.Wrist = hardwareMap.get(Servo.class, "Wrist");
        this.Wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN : position;
        Wrist.setPosition(position);
    }

    public double getCurrentPosition() {
        return Wrist.getPosition();
    }

    public class IntakeWristINIT implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Wrist.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeWristRightPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Wrist.setPosition(AUTO_RIGHT_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeWristLeftPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Wrist.setPosition(AUTO_LEFT_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public Action intakeWristInit(){return new IntakeWristINIT();}
    public Action intakeWristRightPickUp(){return new IntakeWristRightPickUp();}
    public Action intakeWristLeftPickUp(){return new IntakeWristLeftPickUp();}
}

