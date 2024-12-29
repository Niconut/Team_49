package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Elbow {
    private Servo IntakeElbow;

    private  double INIT = 0.5;
    private  double STOW = 0.47;
    private  double DROP = 0.54;
    private  double PICKUP_PREP = 0.555;
    private  double PICKUP = 0.57;

    public enum IntakeElbowState {
        INIT,
        STOW,
        DROP,
        PICKUP_PREP,
        PICKUP
    }

    public Intake_Elbow(HardwareMap hardwareMap) {
        this.IntakeElbow = hardwareMap.get(Servo.class, "Elbow");
        this.IntakeElbow.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){
        IntakeElbow.setPosition(position);}

    public double getCurrentPosition(){return IntakeElbow.getPosition();}

    public void setState(IntakeElbowState state){
        double pos = switch (state){
            case INIT -> INIT;
            case STOW -> STOW;
            case PICKUP -> PICKUP;
            case PICKUP_PREP -> PICKUP_PREP;
            case DROP -> DROP;
        };
        IntakeElbow.setPosition(pos);
    }

}