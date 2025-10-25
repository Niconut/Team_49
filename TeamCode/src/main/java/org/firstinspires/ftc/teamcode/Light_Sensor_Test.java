package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "ColorSensorServoControl", group = "Concept")
public class Light_Sensor_Test extends LinearOpMode {

    private NormalizedColorSensor colorSensor;
    private Servo controlServo;

    // Define servo positions for different colors
    static final double SERVO_POS_RED = 0.2;
    static final double SERVO_POS_BLUE = 0.8;
    static final double SERVO_POS_OTHER = 0.5; // Default or other color position

    @Override
    public void runOpMode() {

        // Initialize hardware
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        controlServo = hardwareMap.get(Servo.class, "control_servo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Convert to HSV for easier color detection
            float hue = JavaUtil.colorToHue(colors.toColor());

            // Determine color based on hue and control servo
            if (hue >= 145 && hue <= 185) { // green is at the beginning and end of the hue spectrum
                telemetry.addData("Detected Color", "Green");
                controlServo.setPosition(SERVO_POS_RED);
                wait(500);
            }
             else {
                telemetry.addData("Detected Color", "Other");
                controlServo.setPosition(SERVO_POS_OTHER);
            }

            telemetry.addData("Hue", "%.3f", hue);
            telemetry.addData("Servo Position", "%.2f", controlServo.getPosition());
            telemetry.update();
        }
    }
}

