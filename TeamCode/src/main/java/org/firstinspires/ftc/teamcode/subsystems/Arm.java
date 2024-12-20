package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Arm {
    private DcMotorEx arm;


    public Arm(HardwareMap hardwareMap) {
        this.arm = hardwareMap.get(DcMotorEx.class, "Arm");

        this.arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.arm.setDirection(DcMotorEx.Direction.FORWARD);
        this.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setPower1(double power) {arm.setPower(power);}

    public int getCurrentPosition() {
        return arm.getCurrentPosition();
        }
}
