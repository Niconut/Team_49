package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Pincer {
    private Servo Pincer;
    public Pincer(HardwareMap hardwareMap) {
        this.Pincer = hardwareMap.get(Servo.class, "Pincer");
        this.Pincer.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double Position){Pincer.setPosition(Position);}

}
