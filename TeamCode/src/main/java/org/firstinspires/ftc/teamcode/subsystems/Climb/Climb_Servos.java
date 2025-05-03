package org.firstinspires.ftc.teamcode.subsystems.Climb;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Climb_Servos extends SubsystemBase {
    private Servo Climb_Servo1;
    private Servo Climb_Servo2;

    private  double INIT = 0.25;
    private double CLIMB = 0.75;
    /*private  double STOW = 0.683;
    private  double DROP = 0.163;
    private  double PICKUP_PREP = 0.1; //0.555;
    private  double PICKUP = 0.05;
    private double PICKUP_DONE = 0.1;
    private double MID = 0.5;
    private double SAFE_MAX = 0.9;
    private double SAFE_MIN = 0;
    private double HAND_OFF = 0.757;//0.515;
    private double HOVER = 0.06;*/

    public enum ClimbSubsystemState {
        INIT,
        CLIMB
        /*STOW,
        DROP,
        PICKUP_PREP,
        PICKUP,
        PICKUP_DONE,
        MID,
        HAND_OFF,
        HOVER*/
    }

    public Climb_Servos(HardwareMap hardwareMap) {
        this.Climb_Servo1 = hardwareMap.get(Servo.class, "Climb_Servo1");
        this.Climb_Servo1.setDirection(Servo.Direction.FORWARD);

        this.Climb_Servo2 = hardwareMap.get(Servo.class, "Climb_Servo2");
        this.Climb_Servo2.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition(double position){
        //position = (position > SAFE_MAX) ? SAFE_MAX : position;
        //position = (position < SAFE_MIN) ? SAFE_MIN: position;
        Climb_Servo1.setPosition(position);
        Climb_Servo2.setPosition(position);
    }


    public double getCurrentPosition(){return Climb_Servo1.getPosition();}

    public void setState(ClimbSubsystemState state){
        double pos = switch (state){
            case INIT -> INIT;
            case CLIMB -> CLIMB;
            /*case STOW -> STOW;
            case PICKUP -> PICKUP;
            case PICKUP_PREP -> PICKUP_PREP;
            case DROP -> DROP;
            case PICKUP_DONE -> PICKUP_DONE;
            case MID -> MID;
            case HAND_OFF -> HAND_OFF;
            case HOVER -> HOVER;*/
        };
        Climb_Servo1.setPosition(pos);
        Climb_Servo2.setPosition(pos);
    }

}