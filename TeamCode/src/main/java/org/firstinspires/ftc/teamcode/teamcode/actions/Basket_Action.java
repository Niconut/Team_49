package org.firstinspires.ftc.teamcode.teamcode.actions;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Basket_Action {
    private Servo Basket;

    public Basket_Action(HardwareMap hardwareMap) {
        this.Basket = hardwareMap.get(Servo.class, "Basket");
        this.Basket.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        Basket.setPosition(position);
    }

    public class Basket_Score implements Action{
        private boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket Packet){
            if (!initialized) {
                Basket.setPosition(0.25);
                initialized = true;
            }
                double pos = Basket.getPosition();
                Packet.put("BasketPos", pos);
                return false;
        }

    }
    public class Basket_Hold implements Action{
        private boolean initialized = false;
                @Override
        public boolean run (@NonNull TelemetryPacket Packet){
                if (!initialized) {
                    Basket.setPosition(0.55);
                    initialized = true;
                }

                double pos = Basket.getPosition();
                Packet.put("BasketPos", pos);
               return false;
            }
        }
    public Action basket_Score(){
        return new Basket_Score();
    }
    public Action basket_Hold(){
        return new Basket_Hold();
    }
}
