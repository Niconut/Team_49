package org.firstinspires.ftc.teamcode.teamcode.tuning;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
//import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.actions.Basket_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Gripper_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Arm_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Viper_Slide_Action;

public final class SplineTest extends LinearOpMode {
    public Basket_Action Basket;
    public Gripper_Action Gripper;
    public Arm_Action Arm;
    public Viper_Slide_Action Viper_Slide;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Basket_Action Basket = new Basket_Action(hardwareMap);
            Gripper_Action Gripper = new Gripper_Action(hardwareMap);
            Arm_Action Arm = new Arm_Action(hardwareMap);
            Viper_Slide_Action Viper_Slide = new Viper_Slide_Action(hardwareMap);

            waitForStart();
            /*Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    Gripper.gripper_Open(),
                                    Arm.armPickup()
                            ),
                           new SequentialAction(
                                    Gripper.gripper_Close(),
                                    Arm.armReset(),
                                    Gripper.gripper_Open(),
                                    Arm.armClear(),
                                    Viper_Slide.viperScore(),
                                    Basket.basket_Score()
                            )
                    )
            );*/
            /*Actions.runBlocking(Gripper.gripper_Close());
            Actions.runBlocking(Basket.basket_Hold());
            Actions.runBlocking(Viper_Slide.viperPickup());
           // sleep(1000);
            Actions.runBlocking(Arm.armPickup());
           // sleep(1000);

          //  sleep(1000);

           // sleep(1000);
           // sleep(1000);
            Actions.runBlocking(Basket.basket_Score());*/
          //  sleep(1000);


                Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                        //.lineToX(24)
                        //.lineToY(24)
                        .splineToConstantHeading(new Vector2d(48, 48), 0) //Math.PI / 2)
                        //.splineTo(new Vector2d(60, 60), Math.PI)
                     .build()
                );
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
