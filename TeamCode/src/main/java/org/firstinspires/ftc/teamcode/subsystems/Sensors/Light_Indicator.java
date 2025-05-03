package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class Light_Indicator extends SubsystemBase {
    private Servo LightSensorLeft;
    private Servo LightSensorRight;

    private static double WHITE = 0.730;
    private static double PURPLE = 0.720;
    private static double BLUE = 0.618;
    private static double GREEN = 0.5;
    private static double YELLOW = 0.345;
    private static double RED = 0.280;
    private static double SAGE = 0.444;

    public enum LightIndicatorState {
        BLUE,
        GREEN,
        YELLOW,
        RED,
        PURPLE,
        WHITE,
        SAGE
    }

    public Light_Indicator(HardwareMap hardwareMap) {
        //this.LightSensorLeft = hardwareMap.get(Servo.class, "LightLeft");
        this.LightSensorRight = hardwareMap.get(Servo.class, "LightRight");
    }

    public void setPosition(double position){
        //LightSensorLeft.setPosition(position);
        LightSensorRight.setPosition(position);
    }

    public void setState(LightIndicatorState state){
        double pos = switch (state){
            case RED -> RED;
            case BLUE -> BLUE;
            case GREEN -> GREEN;
            case YELLOW -> YELLOW;
            case SAGE -> SAGE;
            case WHITE -> WHITE;
            case PURPLE -> PURPLE;
        };
        //LightSensorLeft.setPosition(pos);
        LightSensorRight.setPosition(pos);
    }
}