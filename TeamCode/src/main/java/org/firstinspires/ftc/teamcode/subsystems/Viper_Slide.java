package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class  Viper_Slide {
    private DcMotorEx Viper_Slide;


    public Viper_Slide(HardwareMap hardwareMap) {
        this.Viper_Slide = hardwareMap.get(DcMotorEx.class, "Viper_Slide");

        this.Viper_Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Viper_Slide.setDirection(DcMotorEx.Direction.FORWARD);
        this.Viper_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Viper_Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setPower(double power) {Viper_Slide.setPower(power);}

    public int getCurrentPosition() {
        return Viper_Slide.getCurrentPosition();
        }
    public double getPower() {
        return Viper_Slide.getPower();
    }
}
