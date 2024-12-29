package org.firstinspires.ftc.teamcode.subsystems.intake;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Gripper {
    private Servo IntakeGripper;

    private double INIT = 0.5;
    private static double OPEN = 0.55;
    private static double CLOSE = 0.325;

    public enum IntakeGripperState {
        INIT,
        OPEN,
        CLOSE
    }

    public Intake_Gripper(HardwareMap hardwareMap) {
        this.IntakeGripper = hardwareMap.get(Servo.class, "Intake_Gripper");
        this.IntakeGripper.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        IntakeGripper.setPosition(position);
    }

    public double getCurrentPosition(){return IntakeGripper.getPosition();}

    public void setState(IntakeGripperState state){
        double pos = switch (state){
            case INIT -> INIT;
            case OPEN -> OPEN;
            case CLOSE -> CLOSE;
        };
        IntakeGripper.setPosition(pos);
    }
}
