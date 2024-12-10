package org.firstinspires.ftc.teamcode.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake_Gripper_Action {
    private Servo GripperLeft;
    private Servo GripperRight;

    public Intake_Gripper_Action(HardwareMap hardwareMap) {
        this.GripperLeft = hardwareMap.get(Servo.class, "Gripper_Left");
        this.GripperLeft.setDirection(Servo.Direction.FORWARD);

        this.GripperRight = hardwareMap.get(Servo.class, "Gripper_Right");
        this.GripperRight.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition1(double position){GripperLeft.setPosition(position);}
    public void setPosition2(double position){GripperRight.setPosition(position);}
    public void getPosition1(){GripperRight.getPosition();}
    public void getPosition2(){GripperLeft.getPosition();}

    public class Gripper_Close implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                GripperLeft.setPosition(0.4);
                GripperRight.setPosition(0.625);
                initialized = true;
            }

            double pos = GripperLeft.getPosition();
            Packet.put("Gripper1Pos", pos);
            return false;
        }

    }
    public class Gripper_Open implements Action{
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                GripperLeft.setPosition(0.5);
                GripperRight.setPosition(0.5);
                initialized = true;
            }
            double pos = GripperLeft.getPosition();
            Packet.put("Gripper1Pos", pos);
            return false;
        }

    }

    public Action gripper_Close(){
        return new Gripper_Close();
    }
    public Action gripper_Open(){
        return new Gripper_Open();
    }
}