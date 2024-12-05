package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slide {
    private Servo Slide;

    public Slide(HardwareMap hardwareMap) {
        this.Slide = hardwareMap.get(Servo.class, "Slide");
        this.Slide.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){Slide.setPosition(position);}

    public double getCurrentPosition(){return Slide.getPosition();}
}