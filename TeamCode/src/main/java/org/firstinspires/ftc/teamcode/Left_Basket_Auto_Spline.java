package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Elbow_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Gripper_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Shoulder_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Slide_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Wrist_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Arm_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gripper_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Slide_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

@Autonomous
public final class Left_Basket_Auto_Spline extends LinearOpMode {

    Action  TrajectoryScorePreload,
            TrajectoryPickUpSamples1,
            TrajectoryScorePrep1,
            TrajectoryScoreSamples1,
            TrajectoryPickUpSamples2,
            TrajectoryScorePrep2,
            TrajectoryScoreSamples2,
            TrajectoryPickUpSamples3,
            TrajectoryScorePrep3,
            TrajectoryScoreSamples3,
            TrajectoryPark;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-8.5, -63.5, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Intake_Gripper_Action intakeGripper = new Intake_Gripper_Action(hardwareMap);
        Intake_Wrist_Action intakeWrist = new Intake_Wrist_Action(hardwareMap);
        Intake_Elbow_Action intakeElbow = new Intake_Elbow_Action(hardwareMap);
        Intake_Shoulder_Action intakeShoulder = new Intake_Shoulder_Action(hardwareMap);
        Intake_Slide_Action intakeSlide = new Intake_Slide_Action(hardwareMap);

        Scoring_Gripper_Action scoringGripper = new Scoring_Gripper_Action(hardwareMap);
        Scoring_Arm_Action scoringArm = new Scoring_Arm_Action(hardwareMap);
        Scoring_Slide_Action scoringSlide = new Scoring_Slide_Action(hardwareMap);

        buildTrajectories(drive, beginPose);

        // initialize subsystems
        Actions.runBlocking(
            new SequentialAction(
                scoringGripper.scoringGripperClose(),
                scoringArm.scoringArmInit(),

                intakeGripper.intakeGripperInit(),
                intakeWrist.intakeWristInit(),
                intakeElbow.intakeElbowInit(),
                intakeShoulder.intakeShoulderInit(),
                intakeSlide.intakeSlideInit()
                )
        );

        waitForStart();
            Actions.runBlocking(
                new SequentialAction(
                    scorePreload(scoringGripper, scoringArm, scoringSlide),
                    pickupSample1(intakeGripper, intakeWrist, intakeElbow, intakeShoulder, intakeSlide),
                    HandOff(scoringSlide,scoringArm,scoringGripper,intakeGripper,intakeWrist,intakeElbow,intakeShoulder,intakeSlide),
                    scoreSample1(TrajectoryScoreSamples1, scoringSlide, scoringArm, scoringGripper),
                    pickupSample2(scoringSlide, scoringArm, scoringGripper, intakeGripper, intakeWrist, intakeElbow, intakeShoulder, intakeSlide),
                    HandOff(scoringSlide,scoringArm,scoringGripper,intakeGripper,intakeWrist,intakeElbow,intakeShoulder,intakeSlide),
                    scoreSample2(TrajectoryScoreSamples2, scoringSlide, scoringArm, scoringGripper),
                    pickupSample3(intakeGripper, intakeWrist, intakeElbow, intakeShoulder, intakeSlide),
                    HandOff(scoringSlide,scoringArm,scoringGripper,intakeGripper,intakeWrist,intakeElbow,intakeShoulder,intakeSlide),
                    scoreSample3(TrajectoryScoreSamples3, scoringSlide, scoringArm, scoringGripper)
                    )
                );

    }

    public void buildTrajectories(MecanumDrive drive, Pose2d beginPose) {
        TrajectoryActionBuilder trajectoryHighSpecimenPreload = drive.actionBuilder(beginPose)
                .setReversed(false)
                .lineToY(-35);

        TrajectoryActionBuilder trajectoryPickUpSamples1 = trajectoryHighSpecimenPreload.endTrajectory().fresh()
                .setReversed(true)
                //.strafeToLinearHeading((new Vector2d(-49.5, -48)), Math.toRadians(-90))
                //.strafeToLinearHeading((new Vector2d(-49.5, -38.5)), Math.toRadians(-90));
                .splineToConstantHeading((new Vector2d(-47.5, -46)), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryScoreSamplesPrep1 = trajectoryPickUpSamples1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50,-50), Math.toRadians(-135));


        TrajectoryActionBuilder trajectoryScoreSamples1 = trajectoryScoreSamplesPrep1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(-135));


        TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryScoreSamples1.endTrajectory().fresh()
                //.splineToLinearHeading(new Vector2d(60, -44), Math.toRadians(90));
                //.strafeToLinearHeading(new Vector2d(48, -52), Math.toRadians(90))
                .strafeToLinearHeading((new Vector2d(-57.5, -46)), Math.toRadians(90));
                //.strafeToLinearHeading(new Vector2d(48, -53), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(60, -53), Math.toRadians(90));
                //.splineToConstantHeading(new Vector2d(60, -53), Math.toRadians(90));;

        TrajectoryActionBuilder trajectoryScoreSamplesPrep2 = trajectoryPickUpSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50,-50), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryScoreSamples2 = trajectoryScoreSamplesPrep2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryPickUpSamples3 = trajectoryScoreSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60, -50), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryScoreSamplesPrep3 = trajectoryPickUpSamples3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50,-50), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryScoreSamples3 = trajectoryScoreSamplesPrep3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryPark = trajectoryScoreSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48, -62), Math.toRadians(90));

        TrajectoryScorePreload = trajectoryHighSpecimenPreload.build();
        TrajectoryPickUpSamples1 = trajectoryPickUpSamples1.build();
        TrajectoryScorePrep1 = trajectoryScoreSamplesPrep1.build();
        TrajectoryScoreSamples1 = trajectoryScoreSamples1.build();
        TrajectoryPickUpSamples2 = trajectoryPickUpSamples2.build();
        TrajectoryScorePrep2 = trajectoryScoreSamplesPrep2.build();
        TrajectoryScoreSamples2 = trajectoryScoreSamples2.build();
        TrajectoryPickUpSamples3 = trajectoryPickUpSamples3.build();
        TrajectoryScorePrep3 = trajectoryScoreSamplesPrep3.build();
        TrajectoryScoreSamples3 = trajectoryScoreSamples3.build();
        TrajectoryPark = trajectoryPark.build();
    }

    public Action scorePreload(Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide){
        return new SequentialAction(
            scoringGripper.scoringGripperClose(),
            scoringArm.scoringArmHighChamberScorePrep(),
            scoringSlide.scoringSlideScorePrep(),
            TrajectoryScorePreload,
            new ParallelAction(
                scoringSlide.scoringSlideScore(),
                scoringArm.scoringArmHighChamberScore()
            ),
            new SleepAction(0.25),
            scoringGripper.scoringGripperOpen(),
            scoringSlide.scoringSlideGroundPickUp(),
            scoringArm.scoringArmGroundPickUp()
        );
    }

    public Action pickupSample1(Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide){
        return new SequentialAction(
                TrajectoryPickUpSamples1,
                new ParallelAction(
                        intakeSlide.intakeSlideStow(),
                        intakeShoulder.intakeShoulderPickUpPrep(),
                        intakeElbow.intakeElbowPickUpPrep(),
                        intakeGripper.intakeGripperOpen(),
                        intakeWrist.intakeWristInit()
                ),
                new SleepAction(0.75),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.1),
                intakeElbow.intakeElbowPickUpDone()
        );
    }



    public Action pickupSample2(Scoring_Slide_Action scoringSlide,
                                Scoring_Arm_Action scoringArm,
                                Scoring_Gripper_Action scoringGripper,
                                Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide){
        return new SequentialAction(
                new ParallelAction(
                        intakeElbow.intakeElbowStow(),
                        intakeShoulder.intakeShoulderStow(),
                        intakeSlide.intakeSlideStow()
                ),
                new ParallelAction(
                        TrajectoryPickUpSamples2,
                        intakeSlide.intakeSlideStow(),
                        intakeShoulder.intakeShoulderPickUpPrep(),
                        intakeElbow.intakeElbowPickUpPrep(),
                        intakeGripper.intakeGripperOpen(),
                        intakeWrist.intakeWristInit(),
                        scoringSlide.scoringSlideGroundPickUp(),
                        scoringArm.scoringArmHandOff(),
                        scoringGripper.scoringGripperOpen()
                ),
                new SleepAction(0.75),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.1),
                intakeElbow.intakeElbowPickUpDone()
        );
    }

    public Action pickupSample3(Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide) {
        return new SequentialAction(
                intakeShoulder.intakeShoulderPickUpPrep(),
                //new SleepAction(.5),
                new ParallelAction(
                        TrajectoryPickUpSamples3,
                        new SequentialAction(
                                intakeShoulder.intakeShoulderAutoLeftPickUp(),
                                intakeSlide.intakeSlidePickUpPrep(),
                                intakeWrist.intakeWristLeftPickUp(),
                                intakeElbow.intakeElbowPickUpPrep()
                        )
                ),
                //new SleepAction(0.5),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.1),
                intakeElbow.intakeElbowPickUpDone()
        );
    }

    public Action HandOff(Scoring_Slide_Action scoringSlide,
                          Scoring_Arm_Action scoringArm,
                          Scoring_Gripper_Action scoringGripper,
                          Intake_Gripper_Action intakeGripper,
                          Intake_Wrist_Action intakeWrist,
                          Intake_Elbow_Action intakeElbow,
                          Intake_Shoulder_Action intakeShoulder,
                          Intake_Slide_Action intakeSlide){
        return new SequentialAction(
            new ParallelAction(
                    scoringGripper.scoringGripperOpen(),

                    intakeSlide.intakeSlideHandOffPrep(),
                    intakeElbow.intakeElbowHandOff(),
                    intakeShoulder.intakeShoulderHandOff()
            ),
            new SleepAction(0.3),
            scoringArm.scoringArmHandOff(),
            new SleepAction(1),
            new ParallelAction(
                    scoringArm.scoringArmHandOff(),
                    intakeSlide.intakeSlideHandOff()
            ),
            new SleepAction(0.3),
            scoringGripper.scoringGripperClose(),
            new SleepAction(0.05),
            intakeGripper.intakeGripperOpen(),
            new SleepAction(0.2),
            intakeSlide.intakeSlidePickUpPrep(),
            new SleepAction(0.3),
            new ParallelAction(
                    intakeElbow.intakeElbowStow(),
                    intakeShoulder.intakeShoulderStow(),
                    intakeSlide.intakeSlideStow()
            )
        );
    }

    public Action scoreSample1(Action targetTrajectory,
                             Scoring_Slide_Action scoringSlide,
                              Scoring_Arm_Action scoringArm,
                              Scoring_Gripper_Action scoringGripper) {
        return new SequentialAction(
                new ParallelAction(
                        TrajectoryScorePrep1,
                        scoringArm.scoringArmHighBasketScore(),
                        scoringSlide.scoringSlideHighBasketScore()
                ),
                new SleepAction(0.4),
                targetTrajectory,
                new SleepAction(0.5),
                scoringGripper.scoringGripperOpen(),
                new SleepAction(1),
                scoringArm.scoringArmWallPickUp(),
                new SleepAction(0.5)
        );
    }

    public Action scoreSample2(Action targetTrajectory,
                               Scoring_Slide_Action scoringSlide,
                               Scoring_Arm_Action scoringArm,
                               Scoring_Gripper_Action scoringGripper) {
        return new SequentialAction(
                new ParallelAction(
                        TrajectoryScorePrep2,
                        scoringArm.scoringArmHighBasketScore(),
                        scoringSlide.scoringSlideHighBasketScore()
                ),
                new SleepAction(0.4),
                targetTrajectory,
                new SleepAction(0.5),
                scoringGripper.scoringGripperOpen(),
                new SleepAction(1),
                scoringArm.scoringArmWallPickUp(),
                new SleepAction(0.5)
        );
    }

    public Action scoreSample3(Action targetTrajectory,
                               Scoring_Slide_Action scoringSlide,
                               Scoring_Arm_Action scoringArm,
                               Scoring_Gripper_Action scoringGripper) {
        return new SequentialAction(
                new ParallelAction(
                        TrajectoryScorePrep3,
                        scoringArm.scoringArmHighBasketScore(),
                        scoringSlide.scoringSlideHighBasketScore()
                ),
                new SleepAction(0.4),
                targetTrajectory,
                new SleepAction(0.5),
                scoringGripper.scoringGripperOpen(),
                new SleepAction(1),
                scoringArm.scoringArmWallPickUp(),
                new SleepAction(0.5)
        );
    }




    public Action park(Action targetTrajectory, Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide) {
        return new SequentialAction(
                new ParallelAction(
                        targetTrajectory,
                        scoringSlide.scoringSlideInit(),
                        scoringArm.scoringArmStow()
                ),
                new SleepAction(0.3)
        );
    }

}
