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

        // if running with 2 motors, for example viper slide
        /*
        this.SlideLeft = hardwareMap.get(DcMotorEx.class, "SlideLeft");
        this.SlideRight = hardwareMap.get(DcMotorEx.class, "SlideRight");

        this.SlideLeft.setDirection(DcMotorEx.Direction.FORWARD);
        this.SlideLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.SlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.SlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.SlideRight.setDirection(DcMotorEx.Direction.FORWARD);
        this.SlideRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.SlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.SlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
    }

    public void setPower(double power) {

        Slide.setPower(power);

        // if running with 2 motors, for example viper slide
        //SlideLeft.setPower(power);
        //SlideRight.setPower(power);

    }

    public int getCurrentPosition() {

        return Slide.getCurrentPosition();

        // if running with 2 motors, for example viper slide
        //SlideLeft.getCurrentPosition(); // -> this motor has the encoder connection to control or expansion hub

        }
}
