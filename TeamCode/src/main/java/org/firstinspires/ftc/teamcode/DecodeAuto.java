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

import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Pusher_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Shooter_Action;

@Autonomous
public final class DecodeAuto extends LinearOpMode {
    Action TrajectoryShootBalls1,
            TrajectoryLoadBalls,
            TrajectoryShootBalls2,
            TrajectoryPark;
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-56, 56, Math.toRadians(-45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Scoring_Pusher_Action scoringPusher = new Scoring_Pusher_Action(hardwareMap);

        buildTrajectories(drive, beginPose);
        Actions.runBlocking(
                scoringPusher.StartPusher()
        );

        waitForStart();;
            Actions.runBlocking(
                    new SequentialAction(
                            scoreBalls(TrajectoryShootBalls1, scoringShooter, scoringPusher),
                            humanPlayer(scoringShooter, scoringPusher),
                            scoreBalls(TrajectoryShootBalls2, scoringShooter, scoringPusher),
                            park(scoringShooter, scoringPusher)
                    )

            );
    }

    public void buildTrajectories(MecanumDrive drive, Pose2d beginPose) {
        TrajectoryActionBuilder trajectoryShootBalls1 = drive.actionBuilder(beginPose)
                .setReversed(false)
                .strafeToLinearHeading((new Vector2d(-3, 27)), Math.toRadians(140));

        TrajectoryActionBuilder trajectoryLoadBalls = trajectoryShootBalls1.endTrajectory().fresh()
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(60, -60), Math.toRadians(180));

        TrajectoryActionBuilder trajectoryShootBalls2 = trajectoryLoadBalls.endTrajectory().fresh()
                .strafeToLinearHeading((new Vector2d(-3, 27)), Math.toRadians(140));

        TrajectoryActionBuilder trajectoryPark = trajectoryShootBalls2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(32,-36, Math.toRadians(140)), Math.toRadians(90));

        TrajectoryShootBalls1 = trajectoryShootBalls1.build();
        TrajectoryLoadBalls = trajectoryLoadBalls.build();
        TrajectoryShootBalls2 = trajectoryShootBalls2.build();
        TrajectoryPark = trajectoryPark.build();
    }

    public Action scoreBalls(Action targetTrajectory,
                                Scoring_Shooter_Action scoringShooter,
                               Scoring_Pusher_Action scoringPusher){
        return new SequentialAction(
                new ParallelAction(
                        targetTrajectory,
                        scoringShooter.ShootBall()
                ),
                new SleepAction(0.5),
                scoringPusher.ExtendPusher(),
                new SleepAction(0.5),
                scoringPusher.RetractPusher(),
                new SleepAction(0.5),
                scoringPusher.ExtendPusher(),
                new SleepAction(0.5),
                scoringPusher.RetractPusher(),
                scoringShooter.ZeroBall()
        );
    }
    public Action humanPlayer(Scoring_Shooter_Action scoringShooter,
                               Scoring_Pusher_Action scoringPusher){
        return new SequentialAction(
               TrajectoryLoadBalls,
                new SleepAction(5)
        );
    }


    public Action park(Scoring_Shooter_Action scoringShooter,
                              Scoring_Pusher_Action scoringPusher){
        return new SequentialAction(
                TrajectoryPark,
                new SleepAction(5)
        );
    }

}
