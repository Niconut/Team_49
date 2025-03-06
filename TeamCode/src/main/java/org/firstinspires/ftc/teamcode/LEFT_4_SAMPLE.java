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
//@Disabled
@Autonomous
public final class LEFT_4_SAMPLE extends LinearOpMode {

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
            TrajectoryParkPrep,
            TrajectoryPark;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-39.5, -63.5, Math.toRadians(90));

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
                    pickupSample1(intakeGripper, intakeWrist, intakeElbow, intakeShoulder, intakeSlide, scoringGripper, scoringArm, scoringSlide),
                    HandOff(scoringSlide,scoringArm,scoringGripper,intakeGripper,intakeWrist,intakeElbow,intakeShoulder,intakeSlide),
                    scoreSample1(TrajectoryScoreSamples1, scoringSlide, scoringArm, scoringGripper, intakeElbow, intakeShoulder),
                    pickupSample2(scoringSlide, scoringArm, scoringGripper, intakeGripper, intakeWrist, intakeElbow, intakeShoulder, intakeSlide),
                    HandOff(scoringSlide,scoringArm,scoringGripper,intakeGripper,intakeWrist,intakeElbow,intakeShoulder,intakeSlide),
                    scoreSample2(TrajectoryScoreSamples2, scoringSlide, scoringArm, scoringGripper, intakeElbow, intakeShoulder),
                    pickupSample3(scoringSlide, scoringArm, scoringGripper, intakeGripper, intakeWrist, intakeElbow, intakeShoulder, intakeSlide),
                    HandOff(scoringSlide,scoringArm,scoringGripper,intakeGripper,intakeWrist,intakeElbow,intakeShoulder,intakeSlide),
                    scoreSample3(TrajectoryScoreSamples3, scoringSlide, scoringArm, scoringGripper, intakeElbow, intakeShoulder),
                    park(TrajectoryPark, scoringArm, scoringSlide)
                    )
                );

    }

    public void buildTrajectories(MecanumDrive drive, Pose2d beginPose) {
        TrajectoryActionBuilder trajectoryHighSpecimenPreload = drive.actionBuilder(beginPose)
                .setReversed(false)
                //.lineToY(-35);
                .strafeToLinearHeading(new Vector2d(-53,-53), Math.toRadians(225));

        TrajectoryActionBuilder trajectoryPickUpSamples1 = trajectoryHighSpecimenPreload.endTrajectory().fresh()
                .setReversed(true)
                //.strafeToLinearHeading((new Vector2d(-49.5, -48)), Math.toRadians(-90))
                //.strafeToLinearHeading((new Vector2d(-49.5, -38.5)), Math.toRadians(-90));
                .strafeToLinearHeading((new Vector2d(-50, -45)), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryScoreSamplesPrep1 = trajectoryPickUpSamples1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135));


        TrajectoryActionBuilder trajectoryScoreSamples1 = trajectoryScoreSamplesPrep1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(-135));


        TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryScoreSamples1.endTrajectory().fresh()
                //.splineToLinearHeading(new Vector2d(60, -44), Math.toRadians(90));
                //.strafeToLinearHeading(new Vector2d(48, -52), Math.toRadians(90))
                .strafeToLinearHeading((new Vector2d(-59, -45)), Math.toRadians(90));
                //.strafeToLinearHeading(new Vector2d(48, -53), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(60, -53), Math.toRadians(90));
                //.splineToConstantHeading(new Vector2d(60, -53), Math.toRadians(90));;

        TrajectoryActionBuilder trajectoryScoreSamplesPrep2 = trajectoryPickUpSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryScoreSamples2 = trajectoryScoreSamplesPrep2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryPickUpSamples3 = trajectoryScoreSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-59, -45), Math.toRadians(115));

        TrajectoryActionBuilder trajectoryScoreSamplesPrep3 = trajectoryPickUpSamples3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryScoreSamples3 = trajectoryScoreSamplesPrep3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryParkPrep = trajectoryScoreSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-50, -50), Math. toRadians(-135));

        TrajectoryActionBuilder trajectoryPark = trajectoryScoreSamples3.endTrajectory().fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24,0, Math.toRadians(0)), Math.toRadians(0));

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
        TrajectoryParkPrep = trajectoryParkPrep.build();
        TrajectoryPark = trajectoryPark.build();
    }

    public Action scorePreload(Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide){
        return new SequentialAction(
            scoringGripper.scoringGripperClose(),
            //scoringSlide.scoringSlideScorePrep(),
            new ParallelAction(
                TrajectoryScorePreload,
                scoringArm.scoringArmStow(),
                scoringSlide.scoringSlideHighBasketScore()
            ),
            new SleepAction(0.1),
            scoringArm.scoringArmHighBasketScore(),
            new SleepAction(.5),
            scoringGripper.scoringGripperOpen(),
            scoringArm.scoringArmStow(),
            new SleepAction(0.2)
           /* scoringSlide.scoringSlideGroundPickUp(),
            */
        );
    }

    public Action pickupSample1(Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide,
                                Scoring_Gripper_Action scoringGripper,
                                Scoring_Arm_Action scoringArm,
                                Scoring_Slide_Action scoringSlide
                                ){
        return new SequentialAction(
                TrajectoryPickUpSamples1,
                new SleepAction(0.25),
                new ParallelAction(
                        scoringArm.scoringArmHandOffPrep(),
                        scoringSlide.scoringSlideHandOffPrep(),
                        intakeSlide.intakeSlidePickUpMid(),
                        intakeGripper.intakeGripperOpen(),
                        intakeShoulder.intakeShoulderPickUpPrep(),
                        intakeElbow.intakeElbowHover(),
                        intakeWrist.intakeWristInit()
                ),
                new SleepAction(1),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.2),
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
              /*  new ParallelAction(
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
                        scoringGripper.scoringGripperOpen()
                ),
                new SleepAction(0.75),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.1),
                intakeElbow.intakeElbowPickUpDone()*/
                TrajectoryPickUpSamples2,
                new SleepAction(0.25),
                new ParallelAction(
                        scoringArm.scoringArmHandOffPrep(),
                        scoringSlide.scoringSlideHandOffPrep(),
                        intakeSlide.intakeSlidePickUpMid(),
                        intakeGripper.intakeGripperOpen(),
                        intakeShoulder.intakeShoulderPickUpPrep(),
                        intakeElbow.intakeElbowHover(),
                        intakeWrist.intakeWristInit()
                ),
                new SleepAction(1),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.2),
                intakeElbow.intakeElbowPickUpDone()
        );
    }

    public Action pickupSample3(Scoring_Slide_Action scoringSlide,
                                Scoring_Arm_Action scoringArm,
                                Scoring_Gripper_Action scoringGripper,
                                Intake_Gripper_Action intakeGripper,
                                Intake_Wrist_Action intakeWrist,
                                Intake_Elbow_Action intakeElbow,
                                Intake_Shoulder_Action intakeShoulder,
                                Intake_Slide_Action intakeSlide){
        return new SequentialAction(
              /*  new ParallelAction(
                        intakeElbow.intakeElbowStow(),
                        intakeShoulder.intakeShoulderStow(),
                        intakeSlide.intakeSlideStow(),
                        scoringSlide.scoringSlideGroundPickUp()
                ),
                TrajectoryPickUpSamples3,
                new ParallelAction(
                        intakeSlide.intakeSlideStow(),
                        intakeShoulder.intakeShoulderAutoLeftPickUp(),
                        intakeGripper.intakeGripperOpen(),
                        intakeElbow.intakeElbowPickUpPrep(),
                        intakeWrist.intakeWristLeftPickUp(),

                        scoringGripper.scoringGripperOpen()
                ),

                new SleepAction(0.75),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.1),
                intakeElbow.intakeElbowPickUpDone()*/
                TrajectoryPickUpSamples3,
                new SleepAction(0.25),
                new ParallelAction(
                        scoringArm.scoringArmHandOffPrep(),
                        scoringSlide.scoringSlideHandOffPrep(),
                        intakeSlide.intakeSlideAutoLeftPickUp(),
                        intakeGripper.intakeGripperOpen(),
                        intakeShoulder.intakeShoulderAutoLeftPickUp(),
                        intakeElbow.intakeElbowHover(),
                        intakeWrist.intakeWristLeftPickUp()
                ),
                new SleepAction(1),
                intakeElbow.intakeElbowPickUp(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.2),
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
           /* new ParallelAction(
                    intakeWrist.intakeWristInit(),
                    scoringGripper.scoringGripperOpen(),
                    intakeSlide.intakeSlideHandOffPrep(),
                    intakeElbow.intakeElbowHandOff(),
                    intakeShoulder.intakeShoulderHandOff()
            ),
            new SleepAction(0.5),
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
            )*/
                new ParallelAction(
                        intakeWrist.intakeWristInit(),
                        intakeSlide.intakeSlideHandOff(),
                        intakeElbow.intakeElbowHandOff()
                ),
                new SleepAction(0.025),
                intakeGripper.intakeGripperPartialClose(),
                new SleepAction(0.075),
                intakeGripper.intakeGripperClose(),
                new SleepAction(0.5),
                new ParallelAction(
                scoringSlide.scoringSlideHandOff(),
                scoringArm.scoringArmHandOff()
                ),
                new SleepAction(0.2),
                scoringGripper.scoringGripperClose(),
                new SleepAction(0.2),
                intakeGripper.intakeGripperOpen(),
                new SleepAction(0.1)
        );
    }

    public Action scoreSample1(Action targetTrajectory,
                             Scoring_Slide_Action scoringSlide,
                              Scoring_Arm_Action scoringArm,
                              Scoring_Gripper_Action scoringGripper,
                               Intake_Elbow_Action intakeElbow,
                               Intake_Shoulder_Action intakeShoulder) {
        return new SequentialAction(
               /* new ParallelAction(
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
                scoringGripper.scoringGripperClose(),
                //scoringSlide.scoringSlideScorePrep(),*/
                new ParallelAction(
                        TrajectoryScorePrep1,
                        scoringArm.scoringArmStow(),
                        scoringSlide.scoringSlideHighBasketScore(),
                        intakeElbow.intakeElbowStow(),
                        intakeShoulder.intakeShoulderStow()
                ),
                new SleepAction(0.1),
                scoringArm.scoringArmHighBasketScore(),
                new SleepAction(.5),
                scoringGripper.scoringGripperOpen(),
                scoringArm.scoringArmStow(),
                new SleepAction(0.2)
                /* scoringSlide.scoringSlideGroundPickUp(),
                 */
        );
    }

    public Action scoreSample2(Action targetTrajectory,
                               Scoring_Slide_Action scoringSlide,
                               Scoring_Arm_Action scoringArm,
                               Scoring_Gripper_Action scoringGripper,
                               Intake_Elbow_Action intakeElbow,
                               Intake_Shoulder_Action intakeShoulder) {
        return new SequentialAction(
                /*new ParallelAction(
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
                new SleepAction(0.5)*/
                new ParallelAction(
                        TrajectoryScorePrep2,
                        scoringArm.scoringArmStow(),
                        scoringSlide.scoringSlideHighBasketScore(),
                        intakeElbow.intakeElbowStow(),
                        intakeShoulder.intakeShoulderStow()
                ),
                new SleepAction(0.1),
                scoringArm.scoringArmHighBasketScore(),
                new SleepAction(.5),
                scoringGripper.scoringGripperOpen(),
                scoringArm.scoringArmStow(),
                new SleepAction(0.2)
        );
    }

    public Action scoreSample3(Action targetTrajectory,
                               Scoring_Slide_Action scoringSlide,
                               Scoring_Arm_Action scoringArm,
                               Scoring_Gripper_Action scoringGripper,
                               Intake_Elbow_Action intakeElbow,
                               Intake_Shoulder_Action intakeShoulder) {
        return new SequentialAction(
                /* new ParallelAction(
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
                new SleepAction(0.5)*/
                new ParallelAction(
                        TrajectoryScorePrep3,
                        scoringArm.scoringArmStow(),
                        scoringSlide.scoringSlideHighBasketScore(),
                        intakeElbow.intakeElbowStow(),
                        intakeShoulder.intakeShoulderStow()
                ),
                new SleepAction(0.1),
                scoringArm.scoringArmHighBasketScore(),
                new SleepAction(.5),
                scoringGripper.scoringGripperOpen(),
                scoringArm.scoringArmStow(),
                new SleepAction(0.2)
        );
    }




    public Action park(Action targetTrajectory,
                       Scoring_Arm_Action scoringArm,
                       Scoring_Slide_Action scoringSlide) {
        return new SequentialAction(
                new ParallelAction(
                        targetTrajectory,
                        scoringSlide.scoringSlideInit(),
                        scoringArm.scoringArmStow()
                ),
                new SleepAction(0.5)

        );
    }

}
