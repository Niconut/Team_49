package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Shoulder extends SubsystemBase {
    private Servo Intake_Shoulder;

    private double INIT = 0.858;;
    private double STOW = 0.858;
    private double PICKUP_PREP = 0.5;
    private double DROP = 1;
    private double SAFE_MIN = 0.343;
    private double SAFE_MAX = 1;
    private double MID = 0.5;
    private double PARALLEL = 0.858;
    private double HAND_OFF = 0.5;
    private double SYSCHECK = 0.5;
    private double LEFT_INIT = 0.1;;
    private double LEFT_STOW = 0.1;
    private double LEFT_DROP = 0;

    public enum IntakeShoulderState {
        INIT,
        STOW,
        PICKUP_PREP,
        DROP,
        MID,
        PARALLEL,
        HAND_OFF,
        SYSCHECK,
        LEFT_INIT,
        LEFT_STOW,
        LEFT_DROP
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
            case MID -> MID;
            case PARALLEL -> PARALLEL;
            case HAND_OFF -> HAND_OFF;
            case SYSCHECK -> SYSCHECK;
            case LEFT_INIT -> LEFT_INIT;
            case LEFT_DROP -> LEFT_DROP;
            case LEFT_STOW -> LEFT_STOW;
        };
        Intake_Shoulder.setPosition(pos);
    }
}