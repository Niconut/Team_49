package org.firstinspires.ftc.teamcode.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm_Action {
    public DcMotorEx arm1;
    public PIDController armPID;
    private static int armPickUpPosition = 2150;
    private static int armClearPosition = 550;
    private static int armStartPosition = 0;
    public static double armkP = 0.01;
    public static double armkD = 0.00001;
    public static double armkI = 0.0001;
    private static double armpower = 0;

    public Arm_Action(HardwareMap hardwareMap) {
        this.arm1 = hardwareMap.get(DcMotorEx.class, "arm 1");

        this.arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.arm1.setDirection(DcMotorEx.Direction.FORWARD);
        this.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.armPID = new PIDController(armkP, armkD, armkI);
        this.armPID.setTolerance(50, 10);
    }
    public void setPower1(double power) {arm1.setPower(power);}
    public boolean isAtSetpoint(){
        return armPID.atSetPoint();
    }
    public int getCurrentPosition() {
        return arm1.getCurrentPosition();
        }
    public void setPosition(int setPosition){
        double armPower;
        armPID.setSetPoint(setPosition);
        armPower = armPID.calculate(arm1.getCurrentPosition());
        arm1.setPower(armPower);
    }

    public class ArmPickup implements Action {
        private boolean initialized = false;
        private boolean atSetPoint = armPID.atSetPoint();
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                arm1.setTargetPosition(armPickUpPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(0.8);
                /*
               armPID.setSetPoint(armPickUpPosition);
               while (!armPID.atSetPoint()) {
                   armpower = armPID.calculate(arm1.getCurrentPosition());
                   arm1.setPower(armpower);
               }*/
                initialized = true;
            }

            return false;

        }
    }
    public class ArmReset implements Action {
        private boolean initialized = false;
        private boolean atSetPoint = armPID.atSetPoint();
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                arm1.setTargetPosition(armStartPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(0.8);
                /*
                armPID.setSetPoint(armStartPosition);
                while (!armPID.atSetPoint()) {
                    armpower = armPID.calculate(arm1.getCurrentPosition());
                    arm1.setPower(armpower);

                }*/
                initialized = true;
            }

            return false;

        }
    }
    public class ArmClear implements Action {
        private boolean initialized = false;
        private boolean atSetPoint = armPID.atSetPoint();
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                arm1.setTargetPosition(armClearPosition);
                arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm1.setPower(0.8);

                /*
                armPID.setSetPoint(armClearPosition);
                while (!armPID.atSetPoint()) {
                    armpower = armPID.calculate(arm1.getCurrentPosition());
                    arm1.setPower(armpower);

                }*/
                initialized = true;
            }

            return false;

        }
    }

    public Action armPickup(){
        return new ArmPickup();
    }

    public Action armReset(){
        return new ArmReset();
    }

    public Action armClear(){
        return new ArmClear();
    }
}
