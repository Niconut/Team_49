package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Elbow extends SubsystemBase {
    private Servo IntakeElbow;

    private  double INIT = 0.75;
    private  double STOW = 0.75;
    private  double DROP = 0.275;
    private  double PICKUP_PREP = 0.18; //0.555;
    private  double PICKUP = 0.09;
    private double PICKUP_DONE = 0.18;
    private double MID = 0.5;
    private double SAFE_MAX = 0.76;
    private double SAFE_MIN = 0.08;
    private double HAND_OFF = 0.5;//0.515;
    private double HOVER = 0.13;

    public enum IntakeElbowState {
        INIT,
        STOW,
        DROP,
        PICKUP_PREP,
        PICKUP,
        PICKUP_DONE,
        MID,
        HAND_OFF,
        HOVER
    }

    public Intake_Elbow(HardwareMap hardwareMap) {
        this.IntakeElbow = hardwareMap.get(Servo.class, "Elbow");
        this.IntakeElbow.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN: position;
        IntakeElbow.setPosition(position);}

    public double getCurrentPosition(){return IntakeElbow.getPosition();}

    public void setState(IntakeElbowState state){
        double pos = switch (state){
            case INIT -> INIT;
            case STOW -> STOW;
            case PICKUP -> PICKUP;
            case PICKUP_PREP -> PICKUP_PREP;
            case DROP -> DROP;
            case PICKUP_DONE -> PICKUP_DONE;
            case MID -> MID;
            case HAND_OFF -> HAND_OFF;
            case HOVER -> HOVER;
        };
        IntakeElbow.setPosition(pos);
    }

}