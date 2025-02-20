package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Wrist extends SubsystemBase {
    private Servo Wrist;

    private double INIT = 0.115;
    private double DROP = 0.1;
    private double PICKUP_PREP = 0.5;
    private double STOW = 0.115;
    private double SAFE_MAX = 0.9;
    private double SAFE_MIN = 0.1;
    private double MID = 0.5;
    private double HANDOFF = 0.115;

    public enum IntakeWristState {
        INIT,
        DROP,
        PICKUP_PREP,
        STOW,
        HANDOFF,
        MID
    }

    public Intake_Wrist(HardwareMap hardwareMap) {
        this.Wrist = hardwareMap.get(Servo.class, "Wrist");
        this.Wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        position = (position > SAFE_MAX) ? SAFE_MAX : position;
        position = (position < SAFE_MIN) ? SAFE_MIN : position;
        Wrist.setPosition(position);
    }

    public double getCurrentPosition() {
        return Wrist.getPosition();
    }

    public void setState(IntakeWristState state) {
        double pos = switch (state) {
            case INIT -> INIT;
            case DROP -> DROP;
            case PICKUP_PREP -> PICKUP_PREP;
            case STOW -> STOW;
            case HANDOFF -> HANDOFF;
            case MID -> MID;
        };
        Wrist.setPosition(pos);
    }
}

