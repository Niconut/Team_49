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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous
public final class BackUp_Test_Auto extends LinearOpMode {

    Action  TrajectoryHighSpecimenPrep1,
            TrajectoryPickUpSamples1,
            TrajectoryDropSamples1,
            TrajectoryPickUpSamples2,
            TrajectoryDropSamples2,
            TrajectoryPickUpSamples3,
            TrajectoryDropSamples3,
            TrajectoryHighSpecimenPrep2,
            TrajectoryPickUpSpecimen1,
            TrajectoryHighSpecimenPrep3,
            TrajectoryPickUpSpecimen2,
            TrajectoryHighSpecimenPrep4,
            TrajectoryPark;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(8.5, -63.5, Math.toRadians(90));

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

        waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            // initialize all subsystems
                            scoringArm.scoringArmInit(),
                            scoringGripper.scoringGripperInit(),
                            intakeElbow.intakeElbowInit(),
                            intakeShoulder.intakeShoulderInit(),
                            intakeWrist.intakeWristInit(),
                            intakeSlide.intakeSlideInit(),
                            intakeGripper.intakeGripperInit(),

                            // score preload
                            scoringGripper.scoringGripperClose(),
                            scoringArm.scoringArmHighChamberScorePrep(),
                            scoringSlide.scoringSlideScorePrep(),
                            TrajectoryHighSpecimenPrep1,
                            new ParallelAction(
                                    scoringSlide.scoringSlideScore(),
                                    scoringArm.scoringArmHighChamberScore()
                            ),
                            new SleepAction(0.3),
                            scoringGripper.scoringGripperOpen(),
                            new SleepAction(0.5),
                            scoringSlide.scoringSlideWallPickupPrep(),
                            scoringArm.scoringArmWallPickUpPrep(),

                            // pickup sample # 1
                            new ParallelAction(
                                    TrajectoryPickUpSamples1,
                                    intakeSlide.intakeSlidePickUpPrep(),
                                    intakeShoulder.intakeShoulderParallel(),
                                    intakeElbow.intakeElbowPickUpDone(),
                                    intakeGripper.intakeGripperOpen(),
                                    intakeWrist.intakeWristInit()
                            ),
                            //new SleepAction(0.5),
                            intakeElbow.intakeElbowPickUpPrep(),
                            intakeShoulder.intakeShoulderPickUpPrep(),

                            new SleepAction(1),
                            intakeElbow.intakeElbowPickUp(),
                            new SleepAction(0.2),
                            intakeGripper.intakeGripperClose(),
                            new SleepAction(0.1),
                            intakeElbow.intakeElbowPickUpDone(),

                            // drop sample # 1
                            new ParallelAction(
                                    TrajectoryDropSamples1,
                                    intakeSlide.intakeSlideStow(),
                                    intakeShoulder.intakeShoulderDrop(),
                                    intakeElbow.intakeElbowDrop()
                            ),
                            // new SleepAction(0.5),
                            intakeGripper.intakeGripperOpen(),
                            scoringGripper.scoringGripperOpen(),
                            new SleepAction(0.2),

                            // pickup sample # 2
                            new ParallelAction(
                                    TrajectoryPickUpSamples2,
                                    new SequentialAction(
                                            intakeSlide.intakeSlidePickUpPrep(),
                                            intakeShoulder.intakeShoulderPickUpPrep(),
                                            intakeElbow.intakeElbowPickUpPrep(),
                                            intakeGripper.intakeGripperOpen(),
                                            intakeWrist.intakeWristInit()
                                    )

                            ),
                            new SleepAction(0.5),
                            intakeElbow.intakeElbowPickUp(),
                            new SleepAction(0.2),
                            intakeGripper.intakeGripperClose(),
                            new SleepAction(0.1),
                            intakeElbow.intakeElbowPickUpDone(),

                            // drop sample # 2
                            new ParallelAction(
                                    TrajectoryDropSamples2,
                                    intakeSlide.intakeSlideStow(),
                                    intakeShoulder.intakeShoulderDrop(),
                                    intakeElbow.intakeElbowDrop()
                            ),
                            //new SleepAction(0.5),
                            intakeGripper.intakeGripperOpen(),
                            new SleepAction(0.1),

                            // pickup sample # 3
                            intakeShoulder.intakeShoulderPickUpPrep(),
                            new SleepAction(.5),
                            new ParallelAction(
                                    TrajectoryPickUpSamples3,
                                    new SequentialAction(
                                            intakeShoulder.intakeShoulderAutoRightPickUp(),
                                            intakeSlide.intakeSlidePickUpPrep(),
                                            intakeWrist.intakeWristRightPickUp(),
                                            intakeElbow.intakeElbowPickUpPrep()
                                    )
                            ),
                            //new SleepAction(0.5),
                            intakeElbow.intakeElbowPickUp(),
                            new SleepAction(0.2),
                            intakeGripper.intakeGripperClose(),
                            new SleepAction(0.1),
                            intakeElbow.intakeElbowPickUpDone(),

                            // drop sample # 3
                            new ParallelAction(
                                    TrajectoryDropSamples3,
                                    intakeSlide.intakeSlideStow(),
                                    intakeShoulder.intakeShoulderDrop(),
                                    intakeElbow.intakeElbowDrop()
                            ),
                            //new SleepAction(0.5),
                            intakeGripper.intakeGripperOpen(),

                            // wall pickup # 1
                            scoringSlide.scoringSlideWallPickUp(),
                            scoringArm.scoringArmWallPickUp(),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperClose(),
                            //new SleepAction(0.1),
                            scoringSlide.scoringSlideWallPickUpDone(),

                            // score wall pickup #1
                            //new SleepAction(0.1),
                            scoringSlide.scoringSlideScorePrep2(),
                            scoringArm.scoringArmHighChamberScorePrep(),
                            new SleepAction(0.2),
                            TrajectoryHighSpecimenPrep2,
                            scoringArm.scoringArmHighChamberScore(),
                            scoringSlide.scoringSlideScore(),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperOpen(),
                            //new SleepAction(0.3),


                            new ParallelAction(
                                    TrajectoryPickUpSpecimen1,
                                    scoringSlide.scoringSlideWallPickupPrep(),
                                    scoringArm.scoringArmWallPickUpPrep()
                            ),
                            new SleepAction(0.5),
                            intakeGripper.intakeGripperOpen(),
                            scoringSlide.scoringSlideWallPickUp(),
                            scoringArm.scoringArmWallPickUp(),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperClose(),
                            new ParallelAction(
                                    TrajectoryHighSpecimenPrep3,
                                    scoringSlide.scoringSlideScorePrep2(),
                                    scoringArm.scoringArmHighChamberScorePrep()
                            ),
                            new SleepAction(0.2),
                            scoringArm.scoringArmHighChamberScore(),
                            scoringSlide.scoringSlideScore(),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperOpen(),
                            new ParallelAction(
                                    TrajectoryPickUpSpecimen2,
                                    scoringSlide.scoringSlideWallPickupPrep(),
                                    scoringArm.scoringArmWallPickUpPrep()
                            ),
                            new SleepAction(0.5),
                            intakeGripper.intakeGripperOpen(),
                            scoringSlide.scoringSlideWallPickUp(),
                            scoringArm.scoringArmWallPickUp(),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperClose(),
                            new ParallelAction(
                                    TrajectoryHighSpecimenPrep4,
                                    scoringSlide.scoringSlideScorePrep2(),
                                    scoringArm.scoringArmHighChamberScorePrep()
                            ),
                            new SleepAction(0.2),
                            scoringArm.scoringArmHighChamberScore(),
                            scoringSlide.scoringSlideScore(),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperOpen(),
                            new ParallelAction(
                                    TrajectoryPark,
                                    scoringSlide.scoringSlideInit(),
                                    scoringArm.scoringArmStow()
                            ),
                            new SleepAction(0.2)
                    ));
    }
        public void buildTrajectories(MecanumDrive drive, Pose2d beginPose) {
            TrajectoryActionBuilder trajectoryHighSpecimenPrep1 = drive.actionBuilder(beginPose)
                    .setReversed(false)
                    .lineToY(-35);

            TrajectoryActionBuilder trajectoryPickUpSamples1 = trajectoryHighSpecimenPrep1.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(50, -53), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryDropSamples1 = trajectoryPickUpSamples1.endTrajectory().fresh()
                    .setReversed(true)
                    .strafeToLinearHeading(new Vector2d(48, -62), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryDropSamples1.endTrajectory().fresh()
                    .setReversed(false)
                    .strafeToLinearHeading(new Vector2d(60, -53), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryDropSamples2 = trajectoryPickUpSamples2.endTrajectory().fresh()
                    .setReversed(true)
                    .strafeToLinearHeading(new Vector2d(48, -62), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPickUpSamples3 = trajectoryDropSamples2.endTrajectory().fresh()
                    .setReversed(false)
                    .strafeToLinearHeading(new Vector2d(62, -48), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryDropSamples3 = trajectoryPickUpSamples3.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -62), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryHighSpecimenPrep2 = trajectoryDropSamples3.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(10, -32), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPickUpSpecimen1 = trajectoryHighSpecimenPrep2.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -62), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryHighSpecimenPrep3 = trajectoryPickUpSpecimen1.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(12, -32), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPickUpSpecimen2 = trajectoryHighSpecimenPrep3.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -62), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryHighSpecimenPrep4 = trajectoryPickUpSpecimen2.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(6, -32), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPark = trajectoryHighSpecimenPrep4.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -63), Math.toRadians(90));

            TrajectoryHighSpecimenPrep1 = trajectoryHighSpecimenPrep1.build();
            TrajectoryPickUpSamples1 = trajectoryPickUpSamples1.build();
            TrajectoryDropSamples1 = trajectoryDropSamples1.build();
            TrajectoryPickUpSamples2 = trajectoryPickUpSamples2.build();
            TrajectoryDropSamples2 = trajectoryDropSamples2.build();
            TrajectoryPickUpSamples3 = trajectoryPickUpSamples3.build();
            TrajectoryDropSamples3 = trajectoryDropSamples3.build();
            TrajectoryHighSpecimenPrep2 = trajectoryHighSpecimenPrep2.build();
            TrajectoryPickUpSpecimen1 = trajectoryPickUpSpecimen1.build();
            TrajectoryHighSpecimenPrep3 = trajectoryHighSpecimenPrep3.build();
            TrajectoryPickUpSpecimen2 = trajectoryPickUpSpecimen2.build();
            TrajectoryHighSpecimenPrep4 = trajectoryHighSpecimenPrep4.build();
            TrajectoryPark = trajectoryPark.build();
    }

    public void scorePreload(Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide){
        Actions.runBlocking(
                new SequentialAction(
                        scoringGripper.scoringGripperClose(),
                        scoringArm.scoringArmHighChamberScorePrep(),
                        scoringSlide.scoringSlideScorePrep(),
                        TrajectoryHighSpecimenPrep1,
                        new ParallelAction(
                                scoringSlide.scoringSlideScore(),
                                scoringArm.scoringArmHighChamberScore()
                        ),
                        new SleepAction(0.3),
                        scoringGripper.scoringGripperOpen(),
                        new SleepAction(0.5),
                        scoringSlide.scoringSlideWallPickupPrep(),
                        scoringArm.scoringArmWallPickUpPrep()
                )
        );
    }

    public void wallPickup(Scoring_Gripper_Action scoringGripper, Scoring_Arm_Action scoringArm, Scoring_Slide_Action scoringSlide){

    }

}
