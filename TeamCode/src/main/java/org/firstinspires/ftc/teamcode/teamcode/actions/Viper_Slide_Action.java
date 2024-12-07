package org.firstinspires.ftc.teamcode.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Viper_Slide_Action {
    private DcMotorEx Viper_Slide;
    public static double viperkP = 0.005;
    public static double viperkD= 0.0000;
    public static double viperkI = 0.000;
    private PIDController viperPID;
    private double viperPower = 0;
    private static int viperStartPosition = 0;
    private static int viperHighBasketPosition = -2900; //uses 0.23
    private static int viperLowBasketPosition = -1921; //uses 0.075
    private static int viperHighSpecimenPrepPosition = -1389;
    private static int viperHighSpecimenPosition = -1790;

    public Viper_Slide_Action(HardwareMap hardwareMap) {
        this.Viper_Slide = hardwareMap.get(DcMotorEx.class, "Viper_Slide");

        this.Viper_Slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Viper_Slide.setDirection(DcMotorEx.Direction.FORWARD);
        this.Viper_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Viper_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.viperPID = new PIDController(viperkP, viperkD, viperkI);
        this.viperPID.setTolerance(50, 10);
    }
    public void setPower(double power) {Viper_Slide.setPower(power);}

    public int getCurrentPosition() {
        return Viper_Slide.getCurrentPosition();
        }


    public class ViperScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide.setTargetPosition(viperHighSpecimenPosition);
                Viper_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide.setPower(-0.8);
                initialized = true;
            }

            return false;

        }
    }
    public class ViperPrepScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Viper_Slide.setTargetPosition(viperHighSpecimenPrepPosition);
                Viper_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide.setPower(-0.8);
                initialized = true;
            }

            return false;

        }
    }
    public class ViperStart implements Action {
        private boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Viper_Slide.setTargetPosition(viperStartPosition);
                Viper_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Viper_Slide.setPower(0.8);
                initialized = true;
            }

            return false;

        }
    }
    public Action viperPrepScore(){return new ViperPrepScore();}
    public Action viperScore(){
        return new ViperScore();
    }
    public Action viperStart(){return new ViperStart();}
}
