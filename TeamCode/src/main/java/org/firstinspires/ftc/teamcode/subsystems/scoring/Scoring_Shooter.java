package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring_Shooter extends SubsystemBase {
    private DcMotorEx Wheel1;
    private DcMotorEx Wheel2;

    public double INIT = 0;
    public double OUTTAKE = 3000;


    public enum ScoringShooterState {
        INIT,
        OUTTAKE
    }

    public Scoring_Shooter(final HardwareMap hardwareMap) {
        this.Wheel1 = hardwareMap.get(DcMotorEx.class, "Wheel1");
        this.Wheel1.setDirection(DcMotorSimple.Direction.FORWARD);

        this.Wheel2 = hardwareMap.get(DcMotorEx.class, "Wheel2");
        this.Wheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.Wheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Wheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.Wheel1.setVelocityPIDFCoefficients(100,0.01, 0, 0.5);

        this.Wheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.Wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Wheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.Wheel2.setVelocityPIDFCoefficients(100,0.01, 0, 0.5);
    }

    public void setVelocity(double velocity) {
        Wheel1.setVelocity(velocity);
        Wheel2.setVelocity(velocity);
    }


    public void setState(ScoringShooterState state){
        double vel = switch (state){
            case INIT -> INIT;
            case OUTTAKE -> OUTTAKE;
        };
        Wheel1.setVelocity(vel);
        Wheel2.setVelocity(vel);
    }
}
