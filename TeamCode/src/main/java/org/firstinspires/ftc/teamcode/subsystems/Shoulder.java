package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shoulder {
    private Servo Shoulder;

    public Shoulder(HardwareMap hardwareMap) {
        this.Shoulder = hardwareMap.get(Servo.class, "Shoulder");
        this.Shoulder.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){Shoulder.setPosition(position);}

    public double getCurrentPosition(){return Shoulder.getPosition();}
}