package org.firstinspires.ftc.teamcode.utilandstuff;

import android.os.Environment;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Constants
{
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor armMotor = null;
    public DcMotor wristMotor = null;
    public Servo gripperRight = null;
    public Servo gripperLeft = null;
    public int armDownTargetPosition = 0;
    public int armUpTargetPosition = 8000;
    public int armStartPosition = 0;
    public int armCurrentPosition = 0;
    public  int wristCurrentPosition = 0;
    public int wristStartPosition = 0;
    public int wristUpTargetPosition = 0;
    public int wristScoreTargetPosition = -500;
    public int wristDownTargetPosition = -2200;
    public int gripperleft_Open = 0;
    public int gripperleft_Close = 0;
    public int gripperleftCurrentPosition = 0;
    public double gripperLeftClosedPosition = 0.425;
    public double gripperRightClosedPosition = 0.6;
    public  double gripperLeftOpenPosition = 0.6;
    public  double gripperRightOpenPosition = 0.45;

    public static final double     ARM_UP_SPEED   = 1.0;
    public static final double     ARM_DOWN_SPEED   = 0.5;
    public static final double     WRIST_UP_SPEED = 0.5;
    public static final double     WRIST_DOWN_SPEED = 0.5;
    public static final double     GRIPPER_SPEED = 0.1;
    public double arm_move_power =  0;
    public double wrist_move_power = 0;
    public double gripper_move_power = 0;

    public double wristkP = .01;
    public double wristkD = 0;
    public double wristkI = 0;
    public double armkP = .01;
    public double armkD = 0;
    public  double armkI = 0;
    public  double wristPower = 0;
    public double armPower = 0;
}
