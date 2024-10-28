package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Basket {
    private Servo Basket;

    public Basket(HardwareMap hardwareMap) {
        this.Basket = hardwareMap.get(Servo.class, "Basket");
        this.Basket.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){Basket.setPosition(position);}
}