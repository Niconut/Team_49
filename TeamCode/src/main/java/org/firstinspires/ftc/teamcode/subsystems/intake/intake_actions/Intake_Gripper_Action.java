package org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Gripper_Action {
    private Servo IntakeGripper;

    private static double CLOSE = 0.72;
    private static double OPEN = 0.41;
    private static double INIT = 0.5;

    public Intake_Gripper_Action(HardwareMap hardwareMap) {
        this.IntakeGripper = hardwareMap.get(Servo.class, "Intake_Gripper");
        this.IntakeGripper.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        IntakeGripper.setPosition(position);
    }

    public double getCurrentPosition(){return IntakeGripper.getPosition();}

    public class IntakeGripperOpen implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeGripper.setPosition(OPEN);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeGripperClose implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeGripper.setPosition(CLOSE);
                initialized = true;
            }
            return false;
        }

    }

    public class IntakeGripperInit implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                IntakeGripper.setPosition(INIT);
                initialized = true;
            }
            return false;
        }

    }

    public Action intakeGripperOpen(){return new IntakeGripperOpen();}
    public Action intakeGripperClose(){return new IntakeGripperClose();}
    public Action intakeGripperInit(){return new IntakeGripperInit();}
}
