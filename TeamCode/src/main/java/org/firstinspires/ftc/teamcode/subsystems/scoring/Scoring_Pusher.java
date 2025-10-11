package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Pusher extends SubsystemBase {
    private Servo Scoring_Pusher;

    private double EXTEND = 0.35;
    private double RETRACT = 0.21;

    public enum ScoringPusherState {
        EXTEND,
        RETRACT,
    }

    public Scoring_Pusher(final HardwareMap hardwareMap) {
        this.Scoring_Pusher = hardwareMap.get(Servo.class, "Gripper");
        this.Scoring_Pusher.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        Scoring_Pusher.setPosition(position);
    }

    public double getCurrentPosition(){return Scoring_Pusher.getPosition();}

    public void setState(ScoringPusherState state){
        double pos = switch (state){
            case EXTEND -> EXTEND;
            case RETRACT -> RETRACT;
        };
        Scoring_Pusher.setPosition(pos);
    }
}
