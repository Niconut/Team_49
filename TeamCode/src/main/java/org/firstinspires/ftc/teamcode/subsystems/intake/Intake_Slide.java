package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Slide extends SubsystemBase {
    private Servo Intake_SlideLeft;
    private Servo Intake_SlideRight;

    private static double INIT = 0.6;
    private static double STOW = 0.6;
    private static double DROP = 0.6;
    private static double PICKUP_PREP = 0.5;
    private double SAFE_MIN = 0.14;
    private double HAND_OFF_PREP = 0.5;
    private double HAND_OFF = 0.61;
    private double SYSCHECK = 0.5;

    // if required for expansion limits : 0.73
    // for max extension if passes inspection : 0.85
    // approximately 1.5" delta
    private double SAFE_MAX = .61;

    public enum IntakeSlideState {
        INIT,
        STOW,
        PICKUP_PREP,
        HAND_OFF_PREP,
        HAND_OFF,
        DROP,
        SYSCHECK
    }

    public Intake_Slide(HardwareMap hardwareMap) {
        this.Intake_SlideLeft = hardwareMap.get(Servo.class, "SlideLeft");
        this.Intake_SlideLeft.setDirection(Servo.Direction.FORWARD);

        this.Intake_SlideRight = hardwareMap.get(Servo.class, "SlideRight");
        this.Intake_SlideRight.setDirection(Servo.Direction.REVERSE);
    }

    public double getCurrentPositionLeft(){return Intake_SlideLeft.getPosition();}
    public double getCurrentPositionRight(){return Intake_SlideRight.getPosition();}

    public void setPosition(double position){
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN: position;
        Intake_SlideLeft.setPosition(position);
        Intake_SlideRight.setPosition(position);
    }

    public void setState(IntakeSlideState state){
        double pos = switch (state){
            case INIT -> INIT;
            case STOW -> STOW;
            case DROP -> DROP;
            case PICKUP_PREP -> PICKUP_PREP;
            case HAND_OFF_PREP -> HAND_OFF_PREP;
            case HAND_OFF -> HAND_OFF;
            case SYSCHECK -> SYSCHECK;
        };
        Intake_SlideLeft.setPosition(pos);
        Intake_SlideRight.setPosition(pos);
    }
}