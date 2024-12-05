package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private Servo arm1;
    private Servo arm2;
    public Arm(HardwareMap hardwareMap) {
        this.arm1 = hardwareMap.get(Servo.class, "arm1");
        this.arm1.setDirection(Servo.Direction.FORWARD);

        this.arm2 = hardwareMap.get(Servo.class, "arm2");
        this.arm2.setDirection(Servo.Direction.REVERSE);
    }
    public void setPosition1(double position) {arm1.setPosition(position);}
    public void setPosition2(double position) {arm2.setPosition(position);}

}
