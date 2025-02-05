package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensor_Distance extends SubsystemBase {
    private DistanceSensor Sensor_Distance;

    public Sensor_Distance(final HardwareMap hardwareMap) {
        this.Sensor_Distance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
    }

    public double getRange() {
        return Sensor_Distance.getDistance(DistanceUnit.INCH);
    }
}
