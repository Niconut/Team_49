package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Arm_Action {

    public double INIT = 0.195;
    public double HOME = 0.270;
    public double STOW = 0.400;
    public double GROUND_PICKUP = 0.955;
    public double WALL_PICKUP_PREP = 0.95;
    public double WALL_PICKUP = 0.94;
    public double WALL_PICKUP_RAISE = 0.920;
    public double HIGH_CHAMBER_SCORE_PREP = 0.255;
    public double HIGH_CHAMBER_SCORE = 0.288;
    public double HIGH_BASKET_SCORE_PREP = 0.270;
    public double HIGH_BASKET_SCORE = 0.270;
    public double LOW_BASKET_SCORE_PREP = 0.185;
    public double LOW_BASKET_SCORE = 0.185;
    public double CLIMB_PREP = 0.800;
    public double CLIMB_DONE = 0.800;
    public double HAND_OFF = 0.0;
    public double HANDOFF_PREP = 0.02;
    public double HANDOFF_PREP_EARLY = 0.115;
    public double MID = 0.270;
    public double DIRECT_SCORE = 0.045;
    public double SYSCHECK = 0.270;
    public double LEVEL_1_ASCENT = 0.800;

    /* Gobilda Torque Serv Settings*/
    /*
    private static double INIT = 0.825;
    private static double HOME = 0.8;
    private static double STOW = 0.8;
    private static double GROUND_PICKUP = 0.085;
    private static double WALL_PICKUP_PREP = 0.115;
    private static double WALL_PICKUP = 0.115;
    private static double HIGH_CHAMBER_SCORE_PREP = 0.780;
    private static double HIGH_CHAMBER_SCORE = 0.755;
    private static double HIGH_BASKET_SCORE_PREP = 0.8;
    private static double HIGH_BASKET_SCORE = 0.8;
    private static double LOW_BASKET_SCORE_PREP = 0.9;
    private static double LOW_BASKET_SCORE = 0.9;
    private static double CLIMB_PREP = 0.1;
    private static double CLIMB_DONE = 0.1;
    private static double HAND_OFF = 1;
    private static double HANDOFF_PREP = 1;
    private static double HANDOFF_PREP_EARLY = 0.9;
    private static double MID = 0.5;
    private static double DIRECT_SCORE = 0.988;
    private static double LEVEL_1_ASCENT = 0.1;
     */
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

    public class ScoringArmLevel1Ascent implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(LEVEL_1_ASCENT);
                arm2.setPosition(LEVEL_1_ASCENT);
                initialized = true;
            }
            return false;
        }

    }
    public class ScoringArmHandOffPrep implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                arm1.setPosition(HANDOFF_PREP);
                arm2.setPosition(HANDOFF_PREP);
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
    public Action scoringArmLevel1Ascent(){return new ScoringArmLevel1Ascent();}
    public Action scoringArmHandOffPrep(){return new ScoringArmHandOffPrep();}
}
