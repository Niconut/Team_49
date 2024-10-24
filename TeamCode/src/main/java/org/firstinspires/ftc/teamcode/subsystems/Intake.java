package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Intake {
    private CRServo Intake1;
    private CRServo Intake2;

    public Intake(HardwareMap hardwareMap) {
        this.Intake1 = hardwareMap.get(CRServo.class, "IntakeRight");
        this.Intake1.setDirection(CRServo.Direction.FORWARD);

        this.Intake2 = hardwareMap.get(CRServo.class, "IntakeLeft");
        this.Intake2.setDirection(CRServo.Direction.REVERSE);
    }
    public void setPower1(double power) {
        Intake1.setPower(power);
    }
    public void setPower2(double power){Intake2.setPower(power);}

    public void getPower(){
        Intake1.getPower();
    }
}