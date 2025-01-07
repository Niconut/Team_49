package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Arm_Action {

    public double INIT = 0.8;
    public double HOME = 0.8;
    public double STOW = 0.8;
    public double GROUND_PICKUP = 0.085;
    public double WALL_PICKUP_PREP = 0.1;
    public double WALL_PICKUP = 0.125;
    public double HIGH_CHAMBER_SCORE_PREP = 0.8;
    public double HIGH_CHAMBER_SCORE = 0.75;
    public double HIGH_BASKET_SCORE_PREP = 0.75;
    public double HIGH_BASKET_SCORE = 0.75;
    public double LOW_BASKET_SCORE_PREP = 0.75;
    public double LOW_BASKET_SCORE = 0.75;
    public double HAND_OFF = 0.82;


    private Servo arm1;
    private Servo arm2;

    public Scoring_Arm_Action(HardwareMap hardwareMap) {
        this.arm1 = hardwareMap.get(Servo.class, "arm1");
        this.arm1.setDirection(Servo.Direction.FORWARD);

        this.arm2 = hardwareMap.get(Servo.class, "arm2");
        this.arm2.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double position) {
        arm1.setPosition(position);
        arm2.setPosition(position);
    }

    public double getSoringArm1position(){
        return arm1.getPosition();
    }

    public double getSoringArm2position(){
        return arm2.getPosition();
    }


    public class ScoringArmInit implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(INIT);
                arm2.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmStow implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(STOW);
                arm2.setPosition(STOW);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmGroundPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(GROUND_PICKUP);
                arm2.setPosition(GROUND_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmWallPickUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(WALL_PICKUP);
                arm2.setPosition(WALL_PICKUP);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmWallPickUpPrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(WALL_PICKUP_PREP);
                arm2.setPosition(WALL_PICKUP_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmHighChamberScorePrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(HIGH_CHAMBER_SCORE_PREP);
                arm2.setPosition(HIGH_CHAMBER_SCORE_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmHighChamberScore implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(HIGH_CHAMBER_SCORE);
                arm2.setPosition(HIGH_CHAMBER_SCORE);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmHighBasketScore implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(HIGH_BASKET_SCORE);
                arm2.setPosition(HIGH_BASKET_SCORE);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmHighBasketScorePrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(HIGH_BASKET_SCORE_PREP);
                arm2.setPosition(HIGH_BASKET_SCORE_PREP);
                initialized = true;
            }
            return false;
        }

    }

    public class ScoringArmHandOff implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(HAND_OFF);
                arm2.setPosition(HAND_OFF);
                initialized = true;
            }
            return false;
        }

    }



    public Action scoringArmInit(){return new ScoringArmInit();}
    public Action scoringArmStow(){return new ScoringArmStow();}
    public Action scoringArmGroundPickUp(){return new ScoringArmGroundPickUp();}
    public Action scoringArmWallPickUpPrep(){return new ScoringArmWallPickUpPrep();}
    public Action scoringArmWallPickUp(){return new ScoringArmWallPickUp();}
    public Action scoringArmHighChamberScorePrep(){return new ScoringArmHighChamberScorePrep();}
    public Action scoringArmHighChamberScore(){return new ScoringArmHighChamberScore();}
    public Action scoringArmHighBasketScorePrep(){return new ScoringArmHighBasketScorePrep();}
    public Action scoringArmHighBasketScore(){return new ScoringArmHighBasketScore();}
    public Action scoringArmHandOff(){return new ScoringArmHandOff();}
}
