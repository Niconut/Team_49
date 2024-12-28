package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
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
import org.firstinspires.ftc.teamcode.teamcode.tuning.TuningOpModes;

@Autonomous
public final class TestAuto extends LinearOpMode {

    Action TrajectoryHighSpecimenPrep, TrajectoryPickUpSamples1, TrajectoryPickUpSamples2;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(8.5, -63.5, Math.toRadians(90));

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            TrajectoryActionBuilder trajectoryHighSpecimenPrep = drive.actionBuilder(beginPose)
                    .setReversed(false)
                    .lineToY(-35)
                    .strafeToLinearHeading(new Vector2d(50,-50), Math.toRadians(90))
                    .setReversed(true)
                    .lineToY(-60)
                    .setReversed(false)
                    .strafeToLinearHeading(new Vector2d(60,-50), Math.toRadians(90))
                    .setReversed(true)
                    .lineToY(-60)
                    .setReversed(false)
                    .strafeToLinearHeading(new Vector2d(62,-50), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(10,-34), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(10,-34), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(10,-34), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(90));

            TrajectoryHighSpecimenPrep = trajectoryHighSpecimenPrep.build();

            waitForStart();

            Actions.runBlocking(
                        TrajectoryHighSpecimenPrep
                );
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
