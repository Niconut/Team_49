package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.actions.Arm_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Basket_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Gripper_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Viper_Slide_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Intake_Gripper_Action;
import org.firstinspires.ftc.teamcode.teamcode.tuning.TuningOpModes;
@Autonomous(name="Auto", group="A_DriveCode")
public final class RightAuto extends LinearOpMode {
    public Basket_Action Basket;
    public Gripper_Action Gripper;
    public Arm_Action Arm;
    public Viper_Slide_Action Viper_Slide;
    public Intake_Gripper_Action Intake_Gripper;
    Action TrajectoryHighSpecimenPrep, TrajectoryPickuUpSamples1, TrajectoryPickUpSamples2;
     @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 63.5, Math.toRadians(90));

            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Basket_Action Basket = new Basket_Action(hardwareMap);
            Gripper_Action Gripper = new Gripper_Action(hardwareMap);
            Arm_Action Arm = new Arm_Action(hardwareMap);
            Viper_Slide_Action Viper_Slide = new Viper_Slide_Action(hardwareMap);
            Intake_Gripper_Action Intake_Gripper = new Intake_Gripper_Action(hardwareMap);

          TrajectoryActionBuilder trajectoryHighSpecimenPrep = drive.actionBuilder(beginPose)
          .setReversed(true)
          .lineToY(33);
          //.splineTo(new Vector2d(0,32), Math.toRadians(90))

          TrajectoryActionBuilder trajectoryPickUpSamples1 = drive.actionBuilder(beginPose)
                  .setReversed(false)
                  .lineToY(52)
          .setReversed(true)
          .strafeToLinearHeading(new Vector2d(-49,56), Math.toRadians(-90));

          TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryPickUpSamples1.endTrajectory().fresh()
          .setReversed(false)
         .strafeTo(new Vector2d(-59,56));


          TrajectoryHighSpecimenPrep = trajectoryHighSpecimenPrep.build();
          TrajectoryPickuUpSamples1 = trajectoryPickUpSamples1.build();
          TrajectoryPickUpSamples2 = trajectoryPickUpSamples2.build();

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
                        new SequentialAction(
                        Arm.armClear(),
                        new ParallelAction(
                            TrajectoryHighSpecimenPrep,
                            Viper_Slide.viperPrepScore()
                        ),
                        Viper_Slide.viperScore(),
                        new SleepAction(0.5),
                        Gripper.gripper_Open(),
                        new SleepAction(0.5),
                        new ParallelAction(
                            TrajectoryPickuUpSamples1,
                            Viper_Slide.viperStart()
                        ),
                        new SleepAction(0.1),
                        Arm.armPickup(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Close(),
                        new SleepAction(0.7),
                        Arm.armReset(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Open(),
                        new SleepAction(0.4),
                        Basket.basket_Score(),
                        new SleepAction(0.1),
                        Basket.basket_Hold(),
                        TrajectoryPickUpSamples2,
                        Arm.armPickup(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Close(),
                        new SleepAction(0.7),
                        Arm.armReset(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Open(),
                        new SleepAction(0.4),
                        Basket.basket_Score()
                        )
                );
    }
}
