package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Shoulder extends SubsystemBase {
    private Servo Intake_Shoulder;

    private static double INIT = 0.352;
    private static double STOW = 0.352;
    private static double PICKUP_PREP = 0.5;
    private static double DROP = 0.352;
    private double SAFE_MIN = 0.33;//0.02;
    private double SAFE_MAX = 0.541;//0.8;//0.5411
    private double MID = 0.5;
    private double PARALLEL = 0.5;
    private double HAND_OFF = 0.5;

    public enum IntakeShoulderState {
        INIT,
        STOW,
        PICKUP_PREP,
        DROP,
        MID,
        PARALLEL,
        HAND_OFF
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
        };
        Intake_Shoulder.setPosition(pos);
    }
}