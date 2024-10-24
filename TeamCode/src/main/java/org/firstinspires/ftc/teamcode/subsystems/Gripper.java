package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo Gripper;

    public Gripper(HardwareMap hardwareMap) {
        this.Gripper = hardwareMap.get(Servo.class, "Gripper");
        this.Gripper.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        Gripper.setPosition(position);
    }
    public void getPosition(){Gripper.getPosition();}
}
