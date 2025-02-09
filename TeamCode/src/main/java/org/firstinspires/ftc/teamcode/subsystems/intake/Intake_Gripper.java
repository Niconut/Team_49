package org.firstinspires.ftc.teamcode.subsystems.intake;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Gripper extends SubsystemBase {
    private Servo IntakeGripper;

    private double INIT = 0.5;
    private static double OPEN = 0.45;
    private static double CLOSE = 0.725;
    private static double HAND_OFF_PREP = 0.5;
    private double MID = 0.5;

    public enum IntakeGripperState {
        INIT,
        OPEN,
        CLOSE,
        HAND_OFF_PREP,
        MID
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
            case HAND_OFF_PREP -> HAND_OFF_PREP;
            case MID -> MID;
        };
        IntakeGripper.setPosition(pos);
    }
}
