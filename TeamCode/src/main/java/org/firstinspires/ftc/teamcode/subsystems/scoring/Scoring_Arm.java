package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Arm extends SubsystemBase {

    public double INIT = 0.75;
    public double HOME = 0.8;
    public double STOW = 0.8;
    public double GROUND_PICKUP = 0.085;
    public double WALL_PICKUP_PREP = 0.095;
    public double WALL_PICKUP = 0.095;
    public double HIGH_CHAMBER_SCORE_PREP = 0.780;
    public double HIGH_CHAMBER_SCORE = 0.755;
    public double HIGH_BASKET_SCORE_PREP = 0.75;
    public double HIGH_BASKET_SCORE = 0.75;
    public double LOW_BASKET_SCORE_PREP = 0.75;
    public double LOW_BASKET_SCORE = 0.75;
    public double CLIMB_PREP = 0.1;
    public double CLIMB_DONE = 0.1;
    public double HANDOFF = 0.85;

    public enum ScoringArmState {
        INIT,
        HOME,
        STOW,
        GROUND_PICKUP,
        WALL_PICKUP_PREP,
        WALL_PICKUP,
        HIGH_CHAMBER_SCORE_PREP,
        HIGH_CHAMBER_SCORE,
        HIGH_BASKET_SCORE_PREP,
        HIGH_BASKET_SCORE,
        LOW_BASKET_SCORE_PREP,
        LOW_BASKET_SCORE,
        CLIMB_PREP,
        CLIMB_DONE,
        HANDOFF;
    }

    private Servo arm1;
    private Servo arm2;

    public Scoring_Arm(HardwareMap hardwareMap) {
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

    public void setState(ScoringArmState state){
        double pos = switch (state){
            case INIT -> INIT;
            case HOME -> HOME;
            case STOW -> STOW;
            case GROUND_PICKUP -> GROUND_PICKUP;
            case WALL_PICKUP_PREP -> WALL_PICKUP_PREP;
            case WALL_PICKUP -> WALL_PICKUP;
            case HIGH_CHAMBER_SCORE_PREP -> HIGH_CHAMBER_SCORE_PREP;
            case HIGH_CHAMBER_SCORE -> HIGH_CHAMBER_SCORE;
            case HIGH_BASKET_SCORE_PREP -> HIGH_BASKET_SCORE_PREP;
            case HIGH_BASKET_SCORE -> HIGH_BASKET_SCORE;
            case LOW_BASKET_SCORE_PREP -> LOW_BASKET_SCORE_PREP;
            case LOW_BASKET_SCORE -> LOW_BASKET_SCORE;
            case CLIMB_PREP -> CLIMB_PREP;
            case CLIMB_DONE -> CLIMB_DONE;
            case HANDOFF -> HANDOFF;
        };
        arm1.setPosition(pos);
        arm2.setPosition(pos);
    }

}
