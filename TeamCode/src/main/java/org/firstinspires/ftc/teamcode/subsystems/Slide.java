package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Slide {
    private DcMotorEx Slide;


    public Slide(HardwareMap hardwareMap) {
        this.Slide = hardwareMap.get(DcMotorEx.class, "Slide");

        this.Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Slide.setDirection(DcMotorEx.Direction.FORWARD);
        this.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setPower(double power) {Slide.setPower(power);}

    public int getCurrentPosition() {
        return Slide.getCurrentPosition();
        }
}
