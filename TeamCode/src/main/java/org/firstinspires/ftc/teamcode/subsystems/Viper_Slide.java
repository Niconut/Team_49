package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Viper_Slide {
    private DcMotorEx Viper_Slide1;
    private DcMotorEx Viper_Slide2;


    public Viper_Slide(HardwareMap hardwareMap) {
        this.Viper_Slide1 = hardwareMap.get(DcMotorEx.class, "ViperSlide 1");

        this.Viper_Slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Viper_Slide1.setDirection(DcMotorEx.Direction.REVERSE);
        this.Viper_Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Viper_Slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Viper_Slide2 = hardwareMap.get(DcMotorEx.class, "ViperSlide 2");

        this.Viper_Slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Viper_Slide2.setDirection(DcMotorEx.Direction.FORWARD);
        this.Viper_Slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Viper_Slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setPower1(double power) {Viper_Slide1.setPower(power);}
    public void setPower2(double power) {Viper_Slide2.setPower(power);}
    public int getCurrentPosition() {
        return Viper_Slide2.getCurrentPosition();
    }
}
