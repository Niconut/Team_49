package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Shoulder {
    private Servo Intake_Shoulder;

    private static double INIT = 0.2;
    private static double STOW = 0.2;
    private static double PICKUP_PREP = 0.69;
    private static double DROP = 0.2;
    private double SAFE_MIN = 0.2;
    private double SAFE_MAX = 0.8;

    public enum IntakeShoulderState {
        INIT,
        STOW,
        PICKUP_PREP,
        DROP
    }

    public Intake_Shoulder(HardwareMap hardwareMap) {
        this.Intake_Shoulder = hardwareMap.get(Servo.class, "Shoulder");
        this.Intake_Shoulder.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position){
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN: position;
        Intake_Shoulder.setPosition(position);
    }

    public double getCurrentPosition(){return Intake_Shoulder.getPosition();}

    public void setState(IntakeShoulderState state){
        double pos = switch (state){
            case INIT -> INIT;
            case STOW -> STOW;
            case PICKUP_PREP -> PICKUP_PREP;
            case DROP -> DROP;
        };
        Intake_Shoulder.setPosition(pos);
    }
}