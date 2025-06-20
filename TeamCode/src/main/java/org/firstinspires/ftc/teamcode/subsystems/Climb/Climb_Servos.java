package org.firstinspires.ftc.teamcode.subsystems.Climb;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Climb_Servos extends SubsystemBase {
    private CRServo climbServo1;
    private CRServo climb_Servo2;

    private  double INIT = 0.1;
    private double CLIMB = 1;
    private double RETRACT = -1;
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
        CLIMB,
        RETRACT
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
        this.climbServo1 = hardwareMap.get(CRServo.class, "Climb_Servo1");
        this.climbServo1.setDirection(CRServo.Direction.FORWARD);

        this.climb_Servo2 = hardwareMap.get(CRServo.class, "Climb_Servo2");
        this.climb_Servo2.setDirection(CRServo.Direction.REVERSE);
    }
    public void setPosition(double position){
        //position = (position > SAFE_MAX) ? SAFE_MAX : position;
        //position = (position < SAFE_MIN) ? SAFE_MIN: position;
        climbServo1.setPower(position);
        climb_Servo2.setPower(position);
    }


    public double getCurrentPosition(){return climbServo1.getPower();}

    public void setState(ClimbSubsystemState state){
        double pos = switch (state){
            case INIT -> INIT;
            case CLIMB -> CLIMB;
            case RETRACT -> RETRACT;
            /*case STOW -> STOW;
            case PICKUP -> PICKUP;
            case PICKUP_PREP -> PICKUP_PREP;
            case DROP -> DROP;
            case PICKUP_DONE -> PICKUP_DONE;
            case MID -> MID;
            case HAND_OFF -> HAND_OFF;
            case HOVER -> HOVER;*/
        };
        climbServo1.setPower(pos);
        climb_Servo2.setPower(pos);
    }

}