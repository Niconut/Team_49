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

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Elbow;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Gripper;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Shoulder;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Slide;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Wrist;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Elbow_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Gripper_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Shoulder_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Slide_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_actions.Intake_Wrist_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Arm;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gripper;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Slide;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Arm_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gripper_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Slide_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.tuning.TuningOpModes;

@Autonomous
public final class TestAuto extends LinearOpMode {

    Action TrajectoryHighSpecimenPrep1, TrajectoryPickUpSamples1, TrajectoryDropSamples1, TrajectoryPickUpSamples2,
            TrajectoryDropSamples2, TrajectoryPickUpSamples3, TrajectoryDropSamples3, TrajectoryHighSpecimenPrep2,
            TrajectoryPickUpSpecimen1, TrajectoryHighSpecimenPrep3, TrajectoryPickUpSpecimen2, TrajectoryHighSpecimenPrep4,
            TrajectoryPark;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(8.5, -63.5, Math.toRadians(90));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Intake_Gripper_Action intakeGripper = new Intake_Gripper_Action(hardwareMap);
            Intake_Wrist_Action intakeWrist = new Intake_Wrist_Action(hardwareMap);
            Intake_Elbow_Action intakeElbow = new Intake_Elbow_Action(hardwareMap);
            Intake_Shoulder_Action intakeShoulder = new Intake_Shoulder_Action(hardwareMap);
            Intake_Slide_Action intakeSlide = new Intake_Slide_Action(hardwareMap);

            Scoring_Gripper_Action scoringGripper = new Scoring_Gripper_Action(hardwareMap);
            Scoring_Arm_Action scoringArm = new Scoring_Arm_Action(hardwareMap);
            Scoring_Slide_Action scoringSlide = new Scoring_Slide_Action(hardwareMap);

            TrajectoryActionBuilder trajectoryHighSpecimenPrep1 = drive.actionBuilder(beginPose)
                    .setReversed(false)
                    .lineToY(-35);

            TrajectoryActionBuilder trajectoryPickUpSamples1 = trajectoryHighSpecimenPrep1.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(50,-50), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryDropSamples1 = trajectoryPickUpSamples1.endTrajectory().fresh()
                    .setReversed(true)
                    .lineToY(-60);

            TrajectoryActionBuilder trajectoryPickUpSamples2 = trajectoryDropSamples1.endTrajectory().fresh()
                    .setReversed(false)
                    .strafeToLinearHeading(new Vector2d(60,-50), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryDropSamples2 = trajectoryPickUpSamples2.endTrajectory().fresh()
                    .setReversed(true)
                    .lineToY(-60);

            TrajectoryActionBuilder trajectoryPickUpSamples3 = trajectoryDropSamples2.endTrajectory().fresh()
                    .setReversed(false)
                    .strafeToLinearHeading(new Vector2d(62,-50), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryDropSamples3 = trajectoryPickUpSamples3.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryHighSpecimenPrep2 = trajectoryDropSamples3.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(10,-34), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPickUpSpecimen1 = trajectoryHighSpecimenPrep2.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryHighSpecimenPrep3 = trajectoryPickUpSpecimen1.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(10,-34), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPickUpSpecimen2 = trajectoryHighSpecimenPrep3.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryHighSpecimenPrep4 = trajectoryPickUpSpecimen2.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(10,-34), Math.toRadians(90));

            TrajectoryActionBuilder trajectoryPark = trajectoryHighSpecimenPrep4.endTrajectory().fresh()
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90));

            TrajectoryHighSpecimenPrep1 = trajectoryHighSpecimenPrep1.build();
            TrajectoryPickUpSamples1 = trajectoryPickUpSamples1.build();
            TrajectoryDropSamples1 = trajectoryDropSamples1.build();
            TrajectoryPickUpSamples2 = trajectoryPickUpSamples2.build();
            TrajectoryDropSamples2 = trajectoryHighSpecimenPrep2.build();
            TrajectoryPickUpSamples3 = trajectoryPickUpSamples3.build();
            TrajectoryDropSamples3 = trajectoryDropSamples3.build();
            TrajectoryHighSpecimenPrep2 = trajectoryHighSpecimenPrep2.build();
            TrajectoryPickUpSpecimen1 = trajectoryPickUpSpecimen1.build();
            TrajectoryHighSpecimenPrep3 = trajectoryHighSpecimenPrep3.build();
            TrajectoryPickUpSpecimen2 = trajectoryPickUpSpecimen2.build();
            TrajectoryHighSpecimenPrep4 = trajectoryHighSpecimenPrep4.build();
            TrajectoryPark = trajectoryPark.build();


            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            intakeWrist.intakeWristInit(),
                    scoringGripper.scoringGripperClose(),
                    intakeGripper.intakeGripperOpen(),
                    new ParallelAction(
                        TrajectoryHighSpecimenPrep1,
                            scoringSlide.viperScorePrep(),
                            scoringArm.scoringArmHighChamberScorePrep()
                    ),
                    new SleepAction(0.5),
                    new ParallelAction(
                    scoringSlide.viperScore(),
                    scoringArm.scoringArmHighChamberScore()
                    ),
                    new SleepAction(0.2),
                    scoringGripper.scoringGripperOpen(),
                    new ParallelAction(
                            TrajectoryPickUpSamples1,
                            intakeSlide.intakeSlidePickUpPrep(),
                            new SleepAction(0.2),
                            intakeShoulder.intakeShoulderPickUpPrep(),
                            intakeElbow.intakeElbowPickUpPrep()

                    ),
                    new SleepAction(0.5),
                    intakeElbow.intakeElbowPickUp(),
                    new SleepAction(0.2),
                    intakeGripper.intakeGripperClose(),
                    new SleepAction(0.1),
                    intakeElbow.intakeElbowPickUpDone(),
                    new ParallelAction(
                            TrajectoryDropSamples1,
                            intakeSlide.intakeSlideStow(),
                            new SleepAction(0.2),
                            intakeShoulder.intakeShoulderDrop(),
                            intakeElbow.intakeElbowDrop()

                    ),
                    new SleepAction(0.5),
                    intakeGripper.intakeGripperOpen(),
                    scoringGripper.scoringGripperOpen(),
                    new ParallelAction(
                            TrajectoryPickUpSamples2,
                            intakeSlide.intakeSlidePickUpPrep(),
                            new SleepAction(0.2),
                            intakeShoulder.intakeShoulderPickUpPrep(),
                            intakeElbow.intakeElbowPickUpPrep()

                    ),
                    new SleepAction(0.5),
                    intakeElbow.intakeElbowPickUp(),
                    new SleepAction(0.2),
                    intakeGripper.intakeGripperClose(),
                    new SleepAction(0.1),
                    intakeElbow.intakeElbowPickUpDone(),
                    new ParallelAction(
                            TrajectoryDropSamples2,
                            intakeSlide.intakeSlideStow(),
                            new SleepAction(0.2),
                            intakeShoulder.intakeShoulderDrop(),
                            intakeElbow.intakeElbowDrop()

                    ),
                    new SleepAction(0.5),
                    intakeGripper.intakeGripperOpen(),
                    scoringGripper.scoringGripperOpen(),
                            new ParallelAction(
                                    TrajectoryPickUpSamples3,
                                    intakeSlide.intakeSlidePickUpPrep(),
                                    new SleepAction(0.2),
                                    intakeShoulder.intakeShoulderAutoRightPickUp(),
                                    intakeWrist.intakeWristRightPickUp(),
                                    intakeElbow.intakeElbowPickUpPrep()

                            ),
                    new SleepAction(0.5),
                    intakeElbow.intakeElbowPickUp(),
                    new SleepAction(0.2),
                    intakeGripper.intakeGripperClose(),
                    new SleepAction(0.1),
                    intakeElbow.intakeElbowPickUpDone(),
                    new ParallelAction(
                                    TrajectoryDropSamples3,
                                    intakeSlide.intakeSlideStow(),
                                    new SleepAction(0.2),
                                    intakeShoulder.intakeShoulderDrop(),
                                    intakeElbow.intakeElbowDrop(),
                                    scoringSlide.viperScoreWallPickupPrep(),
                                    scoringArm.scoringArmWallPickUpPrep()
                            ),
                    new SleepAction(0.5),
                    intakeGripper.intakeGripperOpen(),
                    scoringSlide.viperScoreWallPickUp(),
                    scoringArm.scoringArmWallPickUp(),
                    new SleepAction(0.2),
                    scoringGripper.scoringGripperClose(),
                            new ParallelAction(
                                 TrajectoryHighSpecimenPrep2,
                                scoringSlide.viperScorePrep(),
                                 scoringArm.scoringArmHighChamberScorePrep()
                             ),
                    new SleepAction(0.2),
                    scoringGripper.scoringGripperOpen(),
                            new ParallelAction(
                                    TrajectoryPickUpSpecimen1,
                                    scoringSlide.viperScoreWallPickupPrep(),
                                    scoringArm.scoringArmHighBasketScorePrep()
                            ),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperClose(),
                            new ParallelAction(
                                    TrajectoryHighSpecimenPrep3,
                                    scoringSlide.viperScorePrep(),
                                    scoringArm.scoringArmHighChamberScorePrep()
                            ),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperOpen(),
                            new ParallelAction(
                                    TrajectoryPickUpSpecimen2,
                                    scoringSlide.viperScoreWallPickupPrep(),
                                    scoringArm.scoringArmHighBasketScorePrep()
                            ),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperClose(),
                            new ParallelAction(
                                    TrajectoryHighSpecimenPrep4,
                                    scoringSlide.viperScorePrep(),
                                    scoringArm.scoringArmHighChamberScorePrep()
                            ),
                            new SleepAction(0.2),
                            scoringGripper.scoringGripperOpen(),
                            new ParallelAction(
                                    TrajectoryPark,
                                    scoringSlide.viperInit(),
                                    scoringArm.scoringArmStow()
                            ),
                            new SleepAction(0.2)
                ));
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(0))
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
