package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm {
    public DcMotorEx arm1;
    public static double armkP = 0.01;
    public static double armkD = 0.00001;
    public static double armkI = 0.0001;

    public Arm(HardwareMap hardwareMap) {
        this.arm1 = hardwareMap.get(DcMotorEx.class, "arm 1");

        this.arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.arm1.setDirection(DcMotorEx.Direction.FORWARD);
        this.arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDController armPID = new PIDController(armkP, armkI, armkD);
        armPID.setTolerance(50, 10);
    }
    public void setPower1(double power) {arm1.setPower(power);}

    public int getCurrentPosition() {
        return arm1.getCurrentPosition();
        }
}
