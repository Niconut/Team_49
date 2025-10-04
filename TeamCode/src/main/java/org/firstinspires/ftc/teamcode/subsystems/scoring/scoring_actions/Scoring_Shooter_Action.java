package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Scoring_Shooter_Action {
    private DcMotorEx Wheel1;
    private DcMotorEx Wheel2;
    public Scoring_Shooter_Action(HardwareMap hardwareMap) {
        this.Wheel1 = hardwareMap.get(DcMotorEx.class, "Wheel1");
        this.Wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");

        this.Wheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.Wheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Wheel2.setDirection(DcMotorEx.Direction.REVERSE);

        this.Wheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.Wheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public class ShootBall implements Action{
        private boolean initialized = false;
        private double targetvel = 1000;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Wheel1.setPower(0.5);
                Wheel2.setPower(0.5);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class ZeroMotors implements Action{
        private boolean initialized = false;
        private double targetvel = 0;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Wheel1.setPower(0);
                Wheel2.setPower(0);
                initialized = true;
                /*double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public Action ShootBall(){return new ShootBall();}
    public Action ZeroBall(){return new ZeroMotors();}

}
