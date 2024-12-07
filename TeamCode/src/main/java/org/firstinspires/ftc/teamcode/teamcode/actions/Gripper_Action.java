package org.firstinspires.ftc.teamcode.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper_Action {
    private Servo Gripper;

    public Gripper_Action(HardwareMap hardwareMap) {
        this.Gripper = hardwareMap.get(Servo.class, "Gripper");
        this.Gripper.setDirection(Servo.Direction.FORWARD);
    }
    public void setPosition1(double position){Gripper.setPosition(position);}
    public void setPosition2(double position){Gripper.setPosition(position);}
    public void getPosition1(){Gripper.getPosition();}
    public void getPosition2(){Gripper.getPosition();}

    public class Gripper_Open implements Action {
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Gripper.setPosition(0.7);
                initialized = true;
            }

            double pos = Gripper.getPosition();
            Packet.put("Gripper1Pos", pos);
            return false;
        }

    }
    public class Gripper_Close implements Action{
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Gripper.setPosition(0.5);
                initialized = true;
            }
            double pos = Gripper.getPosition();
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