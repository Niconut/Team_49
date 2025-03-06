package org.firstinspires.ftc.teamcode.subsystems.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Wrist extends SubsystemBase {
    private Servo Wrist;

    private double INIT = 0.5;
    private double DROP = 0.3;
    private double PICKUP_PREP = 0.5;
    private double STOW = 0.180;
    private double SAFE_MAX = 0.9;
    private double SAFE_MIN = 0.1;
    private double MID = 0.5;
    private double HANDOFF = 0.5;
    private double SYSCHECK = 0.5;
    private double LEFT_DROP = 0.7;
    private double LEFT_STOW = 0.820;

    public enum IntakeWristState {
        INIT,
        DROP,
        PICKUP_PREP,
        STOW,
        MID,
        HANDOFF,
        SYSCHECK,
        LEFT_DROP,
        LEFT_STOW
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
            case MID -> MID;
            case HANDOFF -> HANDOFF;
            case SYSCHECK -> SYSCHECK;
            case LEFT_DROP -> LEFT_DROP;
            case LEFT_STOW -> LEFT_STOW;
        };
        Wrist.setPosition(pos);
    }
}

