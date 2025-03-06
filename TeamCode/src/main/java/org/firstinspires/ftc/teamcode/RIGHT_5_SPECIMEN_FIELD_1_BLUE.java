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
public final class RIGHT_5_SPECIMEN_FIELD_1_BLUE extends LinearOpMode {



    Action  TrajectoryScorePreload,
            TrajectoryPushSamples,
            TrajectoryPickUpSamples1,
            TrajectoryPickUpSamples1_1,
            TrajectoryPickUpSamples1_2,
            TrajectoryPickUpSamples2,
            TrajectoryPickUpSamples3,
            TrajectoryDropSamples1,
            TrajectoryDropSamples2,
            TrajectoryDropSamples3,
            TrajectoryPickUpWallSpecimen1,
            TrajectoryPickUpWallSpecimen2,
            TrajectoryPickUpWallSpecimen3,
            TrajectoryPickUpWallSpecimen4,
            TrajectoryScoreSpecimen1,
            TrajectoryScoreSpecimen2,
            TrajectoryScoreSpecimen3,
            TrajectoryScoreSpecimen4,
            TrajectoryPark;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(8.5, -63, Math.toRadians(90));

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
                new SleepAction(.2),
                scoringArm.scoringArmInit(),
                new SleepAction(.2),

                intakeGripper.intakeGripperClose(),
                new SleepAction(.2),
                intakeWrist.intakeWristInit(),
                new SleepAction(.2),
                intakeElbow.intakeElbowInit(),
                new SleepAction(.2),
                intakeShoulder.intakeShoulderInit(),
                new SleepAction(.2),
                intakeSlide.intakeSlideInit()
                )
        );

        waitForStart();
            Actions.runBlocking(
                new SequentialAction(
                    scorePreload(scoringGripper, scoringArm, scoringSlide),
                    wallPickup(TrajectoryPushSamples, scoringGripper, scoringArm, scoringSlide),
                    scoreSpecimen(TrajectoryScoreSpecimen1, scoringGripper, scoringArm, scoringSlide),
                    wallPickup(TrajectoryPickUpWallSpecimen2, scoringGripper, scoringArm, scoringSlide),
                    scoreSpecimen(TrajectoryScoreSpecimen2, scoringGripper, scoringArm, scoringSlide),
                    wallPickup(TrajectoryPickUpWallSpecimen3, scoringGripper, scoringArm, scoringSlide),
                    scoreSpecimen(TrajectoryScoreSpecimen3, scoringGripper, scoringArm, scoringSlide),
                    wallPickup(TrajectoryPickUpWallSpecimen4, scoringGripper, scoringArm, scoringSlide),
                    scoreSpecimen(TrajectoryScoreSpecimen4, scoringGripper, scoringArm, scoringSlide),
                    park(TrajectoryPark, scoringGripper, scoringArm, scoringSlide)
                    )
                );

    }

    public void buildTrajectories(MecanumDrive drive, Pose2d beginPose) {
        TrajectoryActionBuilder trajectoryHighSpecimenPreload = drive.actionBuilder(beginPose)
            .setReversed(false)
            .splineToConstantHeading(new Vector2d(7.25,-33.5), Math.toRadians(90));
            //.lineToY(-34);

        TrajectoryActionBuilder trajectoryPushSamples = trajectoryHighSpecimenPreload.endTrajectory().fresh()
            .setReversed(true)
            .setTangent(0)
            .splineToConstantHeading(new Vector2d(33,-31), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(38,-15), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(46,-40, Math.toRadians(90)), Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(44,-49, Math.toRadians(90)), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(40,-30), Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(46,-11, Math.toRadians(90)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(52,-40, Math.toRadians(90)), Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(50,-52, Math.toRadians(90)), Math.toRadians(180))
            .splineToConstantHeading(new Vector2d(44,-30), Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(53.5,-11, Math.toRadians(90)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(63,-36, Math.toRadians(90)), Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(58,-60, Math.toRadians(90)), Math.toRadians(-90))
            //.waitSeconds(0.01)
            .lineToY(-63);

        TrajectoryActionBuilder trajectoryScoreSpecimen1 = trajectoryPushSamples.endTrajectory().fresh()
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(23,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(11.75,-33.5), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryPickUpWallSpecimen2 = trajectoryScoreSpecimen1.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(15, -42, Math.toRadians(90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(33, -52, Math.toRadians(90)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(36.5,-63), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryScoreSpecimen2 = trajectoryPickUpWallSpecimen2.endTrajectory().fresh()
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(15.5,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(9.5,-33.5), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryPickUpSpecimen3 = trajectoryScoreSpecimen2.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14, -42, Math.toRadians(90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(33, -52, Math.toRadians(90)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(36.5,-63), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryScoreSpecimen3 = trajectoryPickUpSpecimen3.endTrajectory().fresh()
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(14,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5,-33.5), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryPickUpSpecimen4 = trajectoryScoreSpecimen3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(13, -42, Math.toRadians(90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(33, -52, Math.toRadians(90)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(36.5,-63), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryScoreSpecimen4 = trajectoryPickUpSpecimen3.endTrajectory().fresh()
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(13,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2.75,-33.5), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryPark = trajectoryScoreSpecimen4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(0,-33), Math.toRadians(90));

        TrajectoryScorePreload = trajectoryHighSpecimenPreload.build();
        TrajectoryPushSamples = trajectoryPushSamples.build();
        TrajectoryPickUpWallSpecimen2 = trajectoryPickUpWallSpecimen2.build();
        TrajectoryPickUpWallSpecimen3 = trajectoryPickUpSpecimen3.build();
        TrajectoryPickUpWallSpecimen4 = trajectoryPickUpSpecimen4.build();
        TrajectoryScoreSpecimen1 = trajectoryScoreSpecimen1.build();
        TrajectoryScoreSpecimen2 = trajectoryScoreSpecimen2.build();
        TrajectoryScoreSpecimen3 = trajectoryScoreSpecimen3.build();
        TrajectoryScoreSpecimen4 = trajectoryScoreSpecimen4.build();
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
            new SleepAction(0.225),
            scoringGripper.scoringGripperOpen(),
            scoringSlide.scoringSlideWallPickupPrep(),
            scoringArm.scoringArmWallPickUpPrep()
        );
    }

    public Action pickupSample1(Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide){
        return new SequentialAction(
            new ParallelAction(
                TrajectoryPickUpSamples1_1,
                intakeSlide.intakeSlidePickUpPrep(),
                intakeShoulder.intakeShoulderParallel(),
                intakeElbow.intakeElbowPickUpDone(),
                intakeGripper.intakeGripperOpen(),
                intakeWrist.intakeWristInit()
            ),
            new ParallelAction(
                TrajectoryPickUpSamples1_2,
                //new SleepAction(0.5),
                intakeElbow.intakeElbowPickUpPrep(),
                intakeShoulder.intakeShoulderPickUpPrep()
            ),

            new SleepAction(0.5),
            intakeElbow.intakeElbowPickUp(),
            new SleepAction(0.2),
            intakeGripper.intakeGripperClose(),
            new SleepAction(0.1),
            intakeElbow.intakeElbowPickUpDone()
        );
    }

    public Action pickupSample1Spline(Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide){
        return new SequentialAction(
                new ParallelAction(
                        TrajectoryPickUpSamples1,
                        intakeSlide.intakeSlideStow(),
                        intakeShoulder.intakeShoulderPickUpPrep(),
                        intakeElbow.intakeElbowPickUpPrep(),
                        intakeGripper.intakeGripperOpen(),
                        intakeWrist.intakeWristInit()
                ),
                new SleepAction(0.35),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.1),
                intakeElbow.intakeElbowPickUpDone()
        );
    }

    public Action pickupSample1Push(Intake_Gripper_Action intakeGripper,
                                      Intake_Wrist_Action intakeWrist,
                                      Intake_Elbow_Action intakeElbow,
                                      Intake_Shoulder_Action intakeShoulder,
                                      Intake_Slide_Action intakeSlide){
        return new SequentialAction(
                        TrajectoryPickUpSamples1
        );
    }

    public Action pickupSample2(Intake_Gripper_Action intakeGripper,
                              Intake_Wrist_Action intakeWrist,
                              Intake_Elbow_Action intakeElbow,
                              Intake_Shoulder_Action intakeShoulder,
                              Intake_Slide_Action intakeSlide) {
        return new SequentialAction(
                        TrajectoryPickUpSamples2
        );
    }

    public Action pickupSample3(Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide) {
        return new SequentialAction(
                //new SleepAction(.5),
                TrajectoryPickUpSamples3
        );
    }

    public Action dropSample(Action targetTrajectory,
                             Intake_Gripper_Action intakeGripper,
                              Intake_Wrist_Action intakeWrist,
                              Intake_Elbow_Action intakeElbow,
                              Intake_Shoulder_Action intakeShoulder,
                              Intake_Slide_Action intakeSlide) {
        return new SequentialAction(
                        targetTrajectory
        );
    }

    public Action wallPickup(Action targetTrajectory, Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide) {
        return new SequentialAction(
                new ParallelAction(
                        targetTrajectory,
                        scoringSlide.scoringSlideWallPickupPrep(),
                        scoringArm.scoringArmWallPickUpPrep()
                ),
                scoringArm.scoringArmWallPickUp(),
                scoringSlide.scoringSlideWallPickUp(),
                //new SleepAction(0.1),
                scoringGripper.scoringGripperAutoScore(),
                new SleepAction(0.1),
                scoringSlide.scoringSlideWallPickUpDone()
        );
    }

    public Action scoreSpecimen(Action targetTrajectory, Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide) {
        return new SequentialAction(
                new ParallelAction(
                    targetTrajectory,
                    new SequentialAction(
                        scoringSlide.scoringSlideScorePrep2(),
                        new SleepAction(0.3),
                        scoringArm.scoringArmHighChamberScorePrep()
                        )
                ),
                //new SleepAction(0.2),
                scoringArm.scoringArmHighChamberScore(),
                scoringSlide.scoringSlideScore(),
                new SleepAction(0.2),
                scoringGripper.scoringGripperOpen()
                //new SleepAction(0.075)
        );
    }

    public Action park(Action targetTrajectory, Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide) {
        return new SequentialAction(
                scoringArm.scoringArmStow(),
                scoringSlide.scoringSlideInit(),


                new SleepAction(0.75),
                new ParallelAction(
                        targetTrajectory,
                        scoringSlide.scoringSlideInit()
                ),
                new SleepAction(0.3)

        );
    }

}
