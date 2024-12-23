package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Scoring_Slide {
    private DcMotorEx Viper_Slide1;
    private DcMotorEx Viper_Slide2;

    PIDController viper_SlidePID = new PIDController(0.01, 0, 0);

    public void setViper_SlidePID(PIDController viper_SlidePID) {
        this.viper_SlidePID = viper_SlidePID;
        viper_SlidePID.setTolerance(10,10);
    }

    private int INIT = 0;
    private int HOME = 25;
    private int STOW = 25;
    private int GROUND_PICKUP = 25;
    private int WALL_PICKUP_PREP = 950;
    private int WALL_PICKUP = 950;
    private int HIGH_CHAMBER_SCORE_PREP = 1300;
    private int HIGH_CHAMBER_SCORE = 1560;
    private int HIGH_BASKET_SCORE_PREP = 4300;
    private int HIGH_BASKET_SCORE = 4300;
    private int LOW_BASKET_SCORE_PREP = 2500;
    private int LOW_BASKET_SCORE = 2500;

    public enum ScoringSlideState {
        INIT,
        HOME,
        STOW,
        GROUND_PICKUP,
        WALL_PICKUP_PREP,
        WALL_PICKUP,
        HIGH_CHAMBER_SCORE_PREP,
        HIGH_CHAMBER_SCORE,
        HIGH_BASKET_SCORE_PREP,
        HIGH_BASKET_SCORE,
        LOW_BASKET_SCORE_PREP,
        LOW_BASKET_SCORE;
    }


    public Scoring_Slide(HardwareMap hardwareMap) {
        this.Viper_Slide1 = hardwareMap.get(DcMotorEx.class, "ViperSlide 1");
        this.Viper_Slide2 = hardwareMap.get(DcMotorEx.class, "ViperSlide 2");

        this.Viper_Slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.Viper_Slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Viper_Slide1.setDirection(DcMotorEx.Direction.FORWARD);
        this.Viper_Slide2.setDirection(DcMotorEx.Direction.REVERSE);

        this.Viper_Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Viper_Slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.Viper_Slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.Viper_Slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.Viper_Slide1.setPositionPIDFCoefficients(0.01);
        this.Viper_Slide2.setPositionPIDFCoefficients(0.01);

        this.setViper_SlidePID(viper_SlidePID);
    }

    public void setPower1(double power) {Viper_Slide1.setPower(power);}
    public void setPower2(double power) {Viper_Slide2.setPower(power);}

    public void setPower(double power) {
        Viper_Slide1.setPower(power);
        Viper_Slide2.setPower(power);
    }

    public int getCurrentPosition() {
        return Viper_Slide2.getCurrentPosition();
    }

    public void setState(ScoringSlideState  state){
        int newpos = switch (state){
            case INIT -> INIT;
            case HOME -> HOME;
            case STOW -> STOW;
            case GROUND_PICKUP -> GROUND_PICKUP;
            case WALL_PICKUP_PREP -> WALL_PICKUP_PREP;
            case WALL_PICKUP -> WALL_PICKUP;
            case HIGH_CHAMBER_SCORE_PREP -> HIGH_CHAMBER_SCORE_PREP;
            case HIGH_CHAMBER_SCORE -> HIGH_CHAMBER_SCORE;
            case HIGH_BASKET_SCORE_PREP -> HIGH_BASKET_SCORE_PREP;
            case HIGH_BASKET_SCORE -> HIGH_BASKET_SCORE;
            case LOW_BASKET_SCORE_PREP -> LOW_BASKET_SCORE_PREP;
            case LOW_BASKET_SCORE -> LOW_BASKET_SCORE;
        };
        viper_SlidePID.setSetPoint(newpos);
        int currentpos = Viper_Slide2.getCurrentPosition();
        double power = viper_SlidePID.calculate(currentpos);
        Viper_Slide1.setPower(power);
        Viper_Slide2.setPower(power);
    }
}

