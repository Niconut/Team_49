package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Scoring_Slide extends SubsystemBase {
    private DcMotorEx Viper_Slide1;
    private DcMotorEx Viper_Slide2;

    private int INIT = 0;
    private int HAND_OFF = 0;
    private int HOME = -20;
    private int STOW = -20;
    private int GROUND_PICKUP = -20;
    private int WALL_PICKUP_PREP = -675;
    private int WALL_PICKUP_DONE = -1000;
    private int WALL_PICKUP = -675;
    private int HIGH_CHAMBER_SCORE_PREP = -850; //860;
    private int HIGH_CHAMBER_SCORE = -1200;
    private int HIGH_BASKET_SCORE_PREP = -2700;
    private int HIGH_BASKET_SCORE = -2700;
    private int LOW_BASKET_SCORE_PREP = -2500;
    private int LOW_BASKET_SCORE = -2500;
    private int CLIMB_PREP = -2200;
    private int CLIMB_DONE = -1200;
    private int SAFE_MIN = -2500;
    private int SAFE_MAX = -20;


    public enum ScoringSlideState {
        INIT,
        HAND_OFF,
        HOME,
        STOW,
        GROUND_PICKUP,
        WALL_PICKUP_PREP,
        WALL_PICKUP_DONE,
        WALL_PICKUP,
        HIGH_CHAMBER_SCORE_PREP,
        HIGH_CHAMBER_SCORE,
        HIGH_BASKET_SCORE_PREP,
        HIGH_BASKET_SCORE,
        LOW_BASKET_SCORE_PREP,
        LOW_BASKET_SCORE,
        CLIMB_PREP,
        CLIMB_DONE;
    }

    public Scoring_Slide(HardwareMap hardwareMap) {
        this.Viper_Slide1 = hardwareMap.get(DcMotorEx.class, "ViperSlide Left");    //  port 0
        this.Viper_Slide2 = hardwareMap.get(DcMotorEx.class, "ViperSlide Right");   //  port 1

        this.Viper_Slide1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.Viper_Slide2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Viper_Slide1.setDirection(DcMotorEx.Direction.FORWARD);
        this.Viper_Slide2.setDirection(DcMotorEx.Direction.REVERSE);

        this.Viper_Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Viper_Slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.Viper_Slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.Viper_Slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //this.Viper_Slide1.setPositionPIDFCoefficients(0.01);
        //this.Viper_Slide2.setPositionPIDFCoefficients(0.01);

        //this.setViper_SlidePID(viper_SlidePID);
    }

    public void setPower1(double power) {Viper_Slide1.setPower(power);}
    public void setPower2(double power) {Viper_Slide2.setPower(power);}

    public void setPower(double power) {
        Viper_Slide1.setPower(power);
        Viper_Slide2.setPower(power);
    }

    public void stopAndResetEncoder(){
        Viper_Slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Viper_Slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Viper_Slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Viper_Slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getLeftCurrentPosition() {
        return Viper_Slide1.getCurrentPosition();
    }

    public int getRightCurrentPosition() {
        return Viper_Slide2.getCurrentPosition();
    }

    public int setState(ScoringSlideState  state){
        int newpos = switch (state){
            case INIT -> INIT;
            case HAND_OFF -> HAND_OFF;
            case HOME -> HOME;
            case STOW -> STOW;
            case GROUND_PICKUP -> GROUND_PICKUP;
            case WALL_PICKUP_PREP -> WALL_PICKUP_PREP;
            case WALL_PICKUP_DONE -> WALL_PICKUP_DONE;
            case WALL_PICKUP -> WALL_PICKUP;
            case HIGH_CHAMBER_SCORE_PREP -> HIGH_CHAMBER_SCORE_PREP;
            case HIGH_CHAMBER_SCORE -> HIGH_CHAMBER_SCORE;
            case HIGH_BASKET_SCORE_PREP -> HIGH_BASKET_SCORE_PREP;
            case HIGH_BASKET_SCORE -> HIGH_BASKET_SCORE;
            case LOW_BASKET_SCORE_PREP -> LOW_BASKET_SCORE_PREP;
            case LOW_BASKET_SCORE -> LOW_BASKET_SCORE;
            case CLIMB_PREP -> CLIMB_PREP;
            case CLIMB_DONE -> CLIMB_DONE;
        };
        newpos = (newpos > SAFE_MAX)? SAFE_MAX:newpos;
        newpos = (newpos < SAFE_MIN)? SAFE_MIN:newpos;
        return newpos;
    }

    public void setStateCommand(ScoringSlideState  state){
        int newpos = switch (state){
            case INIT -> INIT;
            case HAND_OFF -> HAND_OFF;
            case HOME -> HOME;
            case STOW -> STOW;
            case GROUND_PICKUP -> GROUND_PICKUP;
            case WALL_PICKUP_PREP -> WALL_PICKUP_PREP;
            case WALL_PICKUP_DONE -> WALL_PICKUP_DONE;
            case WALL_PICKUP -> WALL_PICKUP;
            case HIGH_CHAMBER_SCORE_PREP -> HIGH_CHAMBER_SCORE_PREP;
            case HIGH_CHAMBER_SCORE -> HIGH_CHAMBER_SCORE;
            case HIGH_BASKET_SCORE_PREP -> HIGH_BASKET_SCORE_PREP;
            case HIGH_BASKET_SCORE -> HIGH_BASKET_SCORE;
            case LOW_BASKET_SCORE_PREP -> LOW_BASKET_SCORE_PREP;
            case LOW_BASKET_SCORE -> LOW_BASKET_SCORE;
            case CLIMB_PREP -> CLIMB_PREP;
            case CLIMB_DONE -> CLIMB_DONE;
        };
        newpos = (newpos > SAFE_MAX)? SAFE_MAX:newpos;
        newpos = (newpos < SAFE_MIN)? SAFE_MIN:newpos;

        Viper_Slide1.setTargetPosition(newpos);
        Viper_Slide2.setTargetPosition(newpos);

        Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Viper_Slide1.setPower(0.6);
        Viper_Slide1.setPower(0.6);
    }

    public boolean isBusy(){
        return (Viper_Slide1.isBusy() || Viper_Slide2.isBusy());
    }
}

