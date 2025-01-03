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
public final class Left_Basket_Auto_Spline extends LinearOpMode {

    Action  TrajectoryScorePreload,
            TrajectoryPickUpSamples1,
            TrajectoryScoreSamples1,
            TrajectoryPickUpSamples2,
            TrajectoryScoreSamples2,
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
                    pickupSample1(scoringSlide, scoringArm, scoringGripper),
                    scoreSample(TrajectoryScoreSamples1, scoringSlide, scoringArm, scoringGripper),
                        pickupSample2(scoringSlide,scoringArm,scoringGripper),
                        scoreSample(TrajectoryScoreSamples2, scoringSlide, scoringArm, scoringGripper)
                    )
                );

    }

    public void buildTrajectories(MecanumDrive drive, Pose2d beginPose) {
        TrajectoryActionBuilder trajectoryHighSpecimenPreload = drive.actionBuilder(beginPose)
                .setReversed(false)
                .lineToY(-36);

        TrajectoryActionBuilder trajectoryPickUpSamples1 = trajectoryHighSpecimenPreload.endTrajectory().fresh()
                .setReversed(true)
                .strafeToLinearHeading((new Vector2d(-49.5, -48)), Math.toRadians(-90))
                .strafeToLinearHeading((new Vector2d(-49.5, -38.5)), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryScoreSamples1 = trajectoryPickUpSamples1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-52, -52), Math.toRadians(-135));


        TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryScoreSamples1.endTrajectory().fresh()
                //.splineToLinearHeading(new Vector2d(60, -44), Math.toRadians(90));
                //.strafeToLinearHeading(new Vector2d(48, -52), Math.toRadians(90))
                .strafeToLinearHeading((new Vector2d(-59.5, -38.5)), Math.toRadians(-90));
                //.strafeToLinearHeading(new Vector2d(48, -53), Math.toRadians(90))
                //.strafeToLinearHeading(new Vector2d(60, -53), Math.toRadians(90));
                //.splineToConstantHeading(new Vector2d(60, -53), Math.toRadians(90));;


        TrajectoryActionBuilder trajectoryScoreSamples2 = trajectoryPickUpSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(-135));

        TrajectoryActionBuilder trajectoryPark = trajectoryScoreSamples2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(48, -62), Math.toRadians(90));

        TrajectoryScorePreload = trajectoryHighSpecimenPreload.build();
        TrajectoryPickUpSamples1 = trajectoryPickUpSamples1.build();
        TrajectoryScoreSamples1 = trajectoryScoreSamples1.build();
        TrajectoryPickUpSamples2 = trajectoryPickUpSamples2.build();
        TrajectoryScoreSamples2 = trajectoryScoreSamples2.build();
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

    public Action pickupSample1(Scoring_Slide_Action scoringSlide,
                                Scoring_Arm_Action scoringArm,
                                Scoring_Gripper_Action scoringGripper){
        return new SequentialAction(
                TrajectoryPickUpSamples1,
                new SleepAction(0.1),
                scoringGripper.scoringGripperClose(),
                new SleepAction(0.1)
        );
    }

    public Action pickupSample2(Scoring_Slide_Action scoringSlide,
                                Scoring_Arm_Action scoringArm,
                                Scoring_Gripper_Action scoringGripper){
        return new SequentialAction(
                new ParallelAction(
                TrajectoryPickUpSamples2,
                scoringSlide.scoringSlideGroundPickUp(),
                scoringArm.scoringArmGroundPickUp(),
                scoringGripper.scoringGripperOpen()
                ),
                new SleepAction(0.5),
                scoringGripper.scoringGripperClose(),
                new SleepAction(0.2)
        );
    }


    public Action scoreSample(Action targetTrajectory,
                             Scoring_Slide_Action scoringSlide,
                              Scoring_Arm_Action scoringArm,
                              Scoring_Gripper_Action scoringGripper) {
        return new SequentialAction(
                new ParallelAction(
                        targetTrajectory,
                        scoringArm.scoringArmHighBasketScore(),
                        scoringSlide.scoringSlideHighBasketScore()
                ),
                new SleepAction(0.3),
                scoringGripper.scoringGripperOpen(),
                new SleepAction(1)
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
