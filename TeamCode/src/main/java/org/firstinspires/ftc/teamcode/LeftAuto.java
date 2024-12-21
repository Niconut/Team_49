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
import org.firstinspires.ftc.teamcode.teamcode.actions.Arm_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Basket_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Gripper_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Intake_Gripper_Action;
import org.firstinspires.ftc.teamcode.teamcode.actions.Viper_Slide_Action;

@Autonomous(name="LeftAuto", group="A_DriveCode")
public final class LeftAuto extends LinearOpMode {
    public Basket_Action Basket;
    public Gripper_Action Gripper;
    public Arm_Action Arm;
    public Viper_Slide_Action Viper_Slide;
    public Intake_Gripper_Action Intake_Gripper;
    Action TrajectoryHighSpecimenPrep, TrajectoryPickuUpSamples1, TrajectoryScore1, TrajectoryPickUpSamples2, TrajectoryScore2, TrajectoryPark;
     @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(10, 62, Math.toRadians(90));

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

          TrajectoryActionBuilder trajectoryPickUpSamples1 = trajectoryHighSpecimenPrep.endTrajectory().fresh()
                  .setReversed(true)
                  .strafeToLinearHeading(new Vector2d(51,48), Math.toRadians(-90));



          TrajectoryActionBuilder trajectoryScore1 = trajectoryPickUpSamples1.endTrajectory().fresh()
                  .setReversed(true)
                  .strafeToLinearHeading(new Vector2d(60,60), Math.toRadians(-187));


          TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryScore1.endTrajectory().fresh()
                  .setReversed(false)
                  .strafeToLinearHeading(new Vector2d(62,47), Math.toRadians(-90));


          TrajectoryActionBuilder trajectoryScore2 = trajectoryPickUpSamples2.endTrajectory().fresh()
                  .setReversed(true)
                  .strafeToLinearHeading(new Vector2d(60,60), Math.toRadians(-187));
          //59,65

         TrajectoryActionBuilder trajectoryPark = trajectoryPickUpSamples2.endTrajectory().fresh()
                 .setReversed(false)
                 .strafeToLinearHeading(new Vector2d(51,48), Math.toRadians(-90));




          TrajectoryHighSpecimenPrep = trajectoryHighSpecimenPrep.build();
          TrajectoryPickuUpSamples1 = trajectoryPickUpSamples1.build();
          TrajectoryScore1 = trajectoryScore1.build();
          TrajectoryPickUpSamples2 = trajectoryPickUpSamples2.build();
          TrajectoryScore2 = trajectoryScore2.build();
          TrajectoryPark = trajectoryPark.build();

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
                        Intake_Gripper.gripper_Open(),
                        Basket.basket_Hold(),
                        Arm.armClear(),
                        new ParallelAction(
                            TrajectoryHighSpecimenPrep,
                            Viper_Slide.viperPrepScore()
                        ),
                        Viper_Slide.viperScore(),
                        new SleepAction(0.5),
                        Gripper.gripper_Open(),
                        new SleepAction(0.5),
                        Intake_Gripper.gripper_Open(),
                            TrajectoryPickuUpSamples1,
                            new SleepAction(0.5),
                            Viper_Slide.viperStart(),
                        new SleepAction(0.1),
                        Arm.armPickup(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Close(),
                        new SleepAction(0.7),
                        Arm.armReset(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Open(),
                        new SleepAction(0.4),
                        Arm.armClear(),
                        new SleepAction(0.3),
                        Viper_Slide.viperHighScore(),
                        new SleepAction(2),
                        TrajectoryScore1,
                        new SleepAction(0.1),
                        Basket.basket_Score(),
                        new SleepAction(1),
                        Basket.basket_Hold(),
                        TrajectoryPickUpSamples2,
                        new SleepAction(0.5),
                        Viper_Slide.viperStart(),
                        new SleepAction(0.1),
                        Arm.armPickup(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Close(),
                        new SleepAction(0.7),
                        Arm.armReset(),
                        new SleepAction(1.1),
                        Intake_Gripper.gripper_Open(),
                        new SleepAction(0.4),
                        Arm.armClear(),
                        new SleepAction(0.5),
                        Viper_Slide.viperHighScore(),
                        new SleepAction(2),
                        TrajectoryScore2,
                        new SleepAction(0.1),
                        Basket.basket_Score(),
                        new SleepAction(1),
                        TrajectoryPark,
                        new SleepAction(1),
                        Viper_Slide.viperStart(),
                        new SleepAction(2),
                        Arm.armReset(),
                        new SleepAction(1)
                        )
                );
    }
}
