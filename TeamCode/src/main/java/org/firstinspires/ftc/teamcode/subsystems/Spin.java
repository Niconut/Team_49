package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Spin {
    private DcMotorEx Spin;


    public Spin(HardwareMap hardwareMap) {
        this.Spin = hardwareMap.get(DcMotorEx.class, "Spin");

        this.Spin.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.Spin.setDirection(DcMotorEx.Direction.FORWARD);
        this.Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setPowerSpin(double power)

    {
        Spin.setPower(power);
    }

    public void function_name (void)
    {

    }

    public int getCurrentPositionSpin() {
        return Spin.getCurrentPosition();
        }
}
