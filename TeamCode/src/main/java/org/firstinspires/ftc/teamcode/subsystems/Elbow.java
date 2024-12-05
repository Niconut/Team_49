package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Elbow {
    private Servo Elbow;

    public Elbow(HardwareMap hardwareMap) {
        this.Elbow = hardwareMap.get(Servo.class, "Elbow");
        this.Elbow.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){Elbow.setPosition(position);}

    public double getCurrentPosition(){return Elbow.getPosition();}
}