package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Gripper_Action {
    private Servo Scoring_Gripper;

    public double CLOSED = 0.3;
    public double OPEN = 0.55;
    public double INIT = 0.55;
    public double AUTO_SCORE = 0.310;

    public Scoring_Gripper_Action(final HardwareMap hardwareMap) {
        this.Scoring_Gripper = hardwareMap.get(Servo.class, "Gripper");
        this.Scoring_Gripper.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        Scoring_Gripper.setPosition(position);
    }

    public double getCurrentPosition(){return Scoring_Gripper.getPosition();}

    public class ScoringGripperInit implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Scoring_Gripper.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringGripperClose implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Scoring_Gripper.setPosition(CLOSED);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringGripperOpen implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Scoring_Gripper.setPosition(OPEN);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringGripperAutoScore implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Scoring_Gripper.setPosition(AUTO_SCORE);
                initialized = true;
            }
            return false;
        }

    }

    public Action scoringGripperInit(){return new ScoringGripperInit();}
    public Action scoringGripperClose(){return new ScoringGripperClose();}
    public Action scoringGripperOpen(){return new ScoringGripperOpen();}
    public Action scoringGripperAutoScore(){return new ScoringGripperAutoScore();}
}
