package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Scoring_Slide_Action {
    private DcMotorEx Viper_Slide1;
    private DcMotorEx Viper_Slide2;

   /* private int INIT = 0;
    private int HOME = -20;
    private int STOW = -20;
    private int GROUND_PICKUP = -20;
    private int WALL_PICKUP_PREP = -675; //-700;
    private int WALL_PICKUP_DONE = -800;
    private int WALL_PICKUP = -650; //-735;
    private int HIGH_CHAMBER_SCORE_PREP = -850;//-810;
    private int HIGH_CHAMBER_SCORE_PREP2 = -850;//-820;
    private int HIGH_CHAMBER_SCORE = -1300;//-1200;
    private int HIGH_BASKET_SCORE_PREP = -2700;
    private int HIGH_BASKET_SCORE = -2700;
    private int LOW_BASKET_SCORE_PREP = -2500;
    private int LOW_BASKET_SCORE = -2500;
    private int SAFE_MIN = -2500;
    private int SAFE_MAX = -20;*/

    private int INIT = 0;
    private int HAND_OFF = 0;
    private int HOME = -30;
    private int PARK = -10;
    private int GROUND_PICKUP = -30;
    private int WALL_PICKUP_PREP = -1000;
    private int WALL_PICKUP_DONE = -1200;
    private int WALL_PICKUP = -1000;
    private int HIGH_CHAMBER_SCORE_PREP = -1150;
    private int HIGH_CHAMBER_SCORE_PREP2 = -1200;//860;
    private int HIGH_CHAMBER_SCORE = -1750;
    private int HIGH_BASKET_SCORE_PREP = -3765;
    private int HIGH_BASKET_SCORE = -3485;
    private int LOW_BASKET_SCORE_PREP = -3485;
    private int LOW_BASKET_SCORE = -3485;
    private int CLIMB_PREP = -3075;
    private int CLIMB_DONE = -1675;
    private int SAFE_MIN = -3485;
    private int SAFE_MAX = -30;


    public Scoring_Slide_Action(HardwareMap hardwareMap) {
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

    public int getLeftCurrentPosition() {
        return Viper_Slide1.getCurrentPosition();
    }

    public int getRightCurrentPosition() {
        return Viper_Slide2.getCurrentPosition();
    }

    public class ScoringSlideScorePrep implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(HIGH_CHAMBER_SCORE_PREP);
                Viper_Slide2.setTargetPosition(HIGH_CHAMBER_SCORE_PREP);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.8);
                Viper_Slide2.setPower(-0.8);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(HIGH_CHAMBER_SCORE);
                Viper_Slide2.setTargetPosition(HIGH_CHAMBER_SCORE);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.8);
                Viper_Slide2.setPower(-0.8);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideWallPickUpPrep implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(WALL_PICKUP_PREP);
                Viper_Slide2.setTargetPosition(WALL_PICKUP_PREP);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.5);
                Viper_Slide2.setPower(-0.5);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideWallPickUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(WALL_PICKUP);
                Viper_Slide2.setTargetPosition(WALL_PICKUP);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.5);
                Viper_Slide2.setPower(-0.5);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideGroundPickUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                int tolerance = 20;
                Viper_Slide1.setTargetPosition(GROUND_PICKUP);
                Viper_Slide2.setTargetPosition(GROUND_PICKUP);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.3);
                Viper_Slide2.setPower(-0.3);

                initialized = (!(Viper_Slide1.getCurrentPosition() < Math.abs(GROUND_PICKUP-tolerance))
                        && !(Viper_Slide2.getCurrentPosition() < Math.abs(GROUND_PICKUP-tolerance)));
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideInit implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(INIT);
                Viper_Slide2.setTargetPosition(INIT);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.3);
                Viper_Slide2.setPower(-0.3);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideHighBasketScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                int tolerance = 20;
                Viper_Slide1.setTargetPosition(HIGH_BASKET_SCORE);
                Viper_Slide2.setTargetPosition(HIGH_BASKET_SCORE);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.8);
                Viper_Slide2.setPower(-0.8);
                initialized = (!(Viper_Slide1.getCurrentPosition() < Math.abs(GROUND_PICKUP-tolerance))
                        && !(Viper_Slide2.getCurrentPosition() < Math.abs(GROUND_PICKUP-tolerance)));
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideWallPickUpDone implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(WALL_PICKUP_DONE);
                Viper_Slide2.setTargetPosition(WALL_PICKUP_DONE);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.5);
                Viper_Slide2.setPower(-0.5);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlideScorePrep2 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(HIGH_CHAMBER_SCORE_PREP2);
                Viper_Slide2.setTargetPosition(HIGH_CHAMBER_SCORE_PREP2);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.8);
                Viper_Slide2.setPower(-0.8);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public class ScoringSlidePark implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide1.setTargetPosition(PARK);
                Viper_Slide2.setTargetPosition(PARK);
                Viper_Slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide1.setPower(-0.3);
                Viper_Slide2.setPower(-0.3);
                initialized = (!Viper_Slide1.isBusy() && !Viper_Slide2.isBusy());
                //initialized = true;
            }

            return false;

        }
    }

    public Action scoringSlideScorePrep(){return new ScoringSlideScorePrep();}
    public Action scoringSlideScore(){return new ScoringSlideScore();}
    public Action scoringSlideWallPickupPrep(){return new ScoringSlideWallPickUpPrep();}
    public Action scoringSlideWallPickUp(){return new ScoringSlideWallPickUp();}
    public Action scoringSlideGroundPickUp(){return new ScoringSlideGroundPickUp();}
    public Action scoringSlideInit(){return new ScoringSlideInit();}
    public Action scoringSlideHighBasketScore(){return new ScoringSlideHighBasketScore();}
    public Action scoringSlideWallPickUpDone(){return new ScoringSlideWallPickUpDone();}
    public Action scoringSlideScorePrep2(){return new ScoringSlideScorePrep2();}
    public Action scoringSlidePark(){return new ScoringSlidePark();}
}

