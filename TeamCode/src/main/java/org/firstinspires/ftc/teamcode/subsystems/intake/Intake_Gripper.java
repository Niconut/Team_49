package org.firstinspires.ftc.teamcode.subsystems.intake;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Gripper extends SubsystemBase {
    private Servo IntakeGripper;

    public double CLOSE = 0.72;
    public double OPEN = 0.41;
    public double INIT = 0.5;
    public double MID = 0.5;
    public double PARTIAL_CLOSE = 0.665;
    public double HAND_OFF_PREP = 0.665;
    public double SYSCHECK = 0.5;

    public enum IntakeGripperState {
        INIT,
        OPEN,
        CLOSE,
        HAND_OFF_PREP,
        MID,
        PARTIAL_CLOSE,
        SYSCHECK
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
            case PARTIAL_CLOSE -> PARTIAL_CLOSE;
            case SYSCHECK -> SYSCHECK;
        };
        IntakeGripper.setPosition(pos);
    }
}
