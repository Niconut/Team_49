package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Specimen_Gripper {
    private Servo Gripper;

    public Specimen_Gripper(HardwareMap hardwareMap) {

        this.Gripper = hardwareMap.get(Servo.class, "Gripper");
        this.Gripper.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition(double position){Gripper.setPosition(position);}
    public void getPosition1(){Gripper.getPosition();}
}
