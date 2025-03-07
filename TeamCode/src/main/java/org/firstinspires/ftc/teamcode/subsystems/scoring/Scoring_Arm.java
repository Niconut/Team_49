package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Arm extends SubsystemBase {

    /* Gobilda Torque Servo Settings */
    /*
    public double INIT = 0.825;
    public double HOME = 0.8;
    public double STOW = 0.8;
    public double GROUND_PICKUP = 0.085;
    public double WALL_PICKUP_PREP = 0.115;
    public double WALL_PICKUP = 0.115;
    public double HIGH_CHAMBER_SCORE_PREP = 0.780;
    public double HIGH_CHAMBER_SCORE = 0.755;
    public double HIGH_BASKET_SCORE_PREP = 0.8;
    public double HIGH_BASKET_SCORE = 0.8;
    public double LOW_BASKET_SCORE_PREP = 0.9;
    public double LOW_BASKET_SCORE = 0.9;
    public double CLIMB_PREP = 0.1;
    public double CLIMB_DONE = 0.1;
    public double HANDOFF = 1;
    public double HANDOFF_PREP = 1;
    public double HANDOFF_PREP_EARLY = 0.9;
    public double MID = 0.5;
    public double DIRECT_SCORE = 0.988;
    public double SYSCHECK = 0.75;
    */

    /* SWYFT Balanced Servo Settings Default Options*/

    public double INIT = 0.175;
    public double HOME = 0.270;
    public double STOW = 0.270;
    public double GROUND_PICKUP = 0.960;
    public double WALL_PICKUP_PREP = 0.950;
    public double WALL_PICKUP = 0.940;
    public double WALL_PICKUP_RAISE = 0.910;
    public double HIGH_CHAMBER_SCORE_PREP = 0.260;
    public double HIGH_CHAMBER_SCORE = 0.300; //0.288;
    public double HIGH_BASKET_SCORE_PREP = 0.270;
    public double HIGH_BASKET_SCORE = 0.270;
    public double LOW_BASKET_SCORE_PREP = 0.185;
    public double LOW_BASKET_SCORE = 0.185;
    public double CLIMB_PREP = 0.800;
    public double CLIMB_DONE = 0.800;
    public double HANDOFF = 0.000;
    public double HANDOFF_PREP = 0.020;
    public double HANDOFF_PREP_EARLY = 0.100;
    public double MID = 0.270;
    public double DIRECT_SCORE = 0.080;
    public double SYSCHECK = 0.280;

    public enum ScoringArmState {
        INIT,
        HOME,
        STOW,
        GROUND_PICKUP,
        WALL_PICKUP_PREP,
        WALL_PICKUP,
        WALL_PICKUP_RAISE,
        HIGH_CHAMBER_SCORE_PREP,
        HIGH_CHAMBER_SCORE,
        HIGH_BASKET_SCORE_PREP,
        HIGH_BASKET_SCORE,
        LOW_BASKET_SCORE_PREP,
        LOW_BASKET_SCORE,
        CLIMB_PREP,
        CLIMB_DONE,
        HANDOFF,
        HANDOFF_PREP,
        HANDOFF_PREP_EARLY,
        MID,
        DIRECT_SCORE,
        SYSCHECK
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
            case WALL_PICKUP_RAISE -> WALL_PICKUP_RAISE;
            case HIGH_CHAMBER_SCORE_PREP -> HIGH_CHAMBER_SCORE_PREP;
            case HIGH_CHAMBER_SCORE -> HIGH_CHAMBER_SCORE;
            case HIGH_BASKET_SCORE_PREP -> HIGH_BASKET_SCORE_PREP;
            case HIGH_BASKET_SCORE -> HIGH_BASKET_SCORE;
            case LOW_BASKET_SCORE_PREP -> LOW_BASKET_SCORE_PREP;
            case LOW_BASKET_SCORE -> LOW_BASKET_SCORE;
            case CLIMB_PREP -> CLIMB_PREP;
            case CLIMB_DONE -> CLIMB_DONE;
            case HANDOFF -> HANDOFF;
            case HANDOFF_PREP -> HANDOFF_PREP;
            case HANDOFF_PREP_EARLY -> HANDOFF_PREP_EARLY;
            case MID -> MID;
            case DIRECT_SCORE -> DIRECT_SCORE;
            case SYSCHECK -> SYSCHECK;
        };
        arm1.setPosition(pos);
        arm2.setPosition(pos);
    }

}
