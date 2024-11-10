package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ClimbArm {
    private DcMotorEx ClimbArm;


    public ClimbArm(HardwareMap hardwareMap) {
        this.ClimbArm = hardwareMap.get(DcMotorEx.class, "ClimbArm");

        this.ClimbArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.ClimbArm.setDirection(DcMotorEx.Direction.FORWARD);
        this.ClimbArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.ClimbArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setPower(double power) {ClimbArm.setPower(power);}

    public int getCurrentPosition() {
        return ClimbArm.getCurrentPosition();
        }
}
