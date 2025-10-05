package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -63.5, Math.toRadians(90)))
                .setReversed(false)
                .lineToY(-36)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(48,-46), Math.toRadians(90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(46, -58), Math.toRadians(45))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(57,-46), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(46, -58), Math.toRadians(45))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(50, -50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, -50), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(42, -58), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(36, -64), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-4, -34), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36, -64), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(36, -64), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(4, -34), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(44, -64), Math.toRadians(90))
                .build());

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySecondBot.runAction(mySecondBot.getDrive().actionBuilder(new Pose2d(-10, 63.5, Math.toRadians(-90)))
                .setReversed(false)
                .lineToY(36)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48,46), Math.toRadians(-90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-46, 58), Math.toRadians(-45))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-57,46), Math.toRadians(-90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-46, 58), Math.toRadians(-45))
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-50, 50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-60, 50), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-42, 58), Math.toRadians(-45))
                .strafeToLinearHeading(new Vector2d(-36, 64), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(4, 34), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-36, 64), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(0, 34), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-36, 64), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-4, 34), Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(-44, 64), Math.toRadians(-90))
                .build());

        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myThirdBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, -62, Math.toRadians(90)))
                .lineToY(-35)
                .setReversed(true)
                .splineTo(new Vector2d(-48,-34), Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(-57,-57), Math.toRadians(-135))
                .setReversed(true)
                .splineTo(new Vector2d(-58,-34), Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(-57,-57), Math.toRadians(-135))
                .strafeToLinearHeading(new Vector2d(-52,-10), Math.toRadians(0))
                .setReversed(true)
                .lineToX(-25)
                .build());


        RoadRunnerBotEntity myFourthBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myFourthBot.runAction(myThirdBot.getDrive().actionBuilder(new Pose2d(-56, 56, Math.toRadians(-45)))
                .setReversed(false)
                .strafeToLinearHeading((new Vector2d(-3, 27)), Math.toRadians(140))
                .strafeToSplineHeading(new Vector2d(60, -60), Math.toRadians(180))
                .strafeToLinearHeading((new Vector2d(-3, 27)), Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(32,-36, Math.toRadians(140)), Math.toRadians(90))
                .build());
                //.splineToLinearHeading(new Pose2d(-3,27, Math.toRadians(135)), Math.toRadians(140))
                /*.setReversed(true)

                //.strafeToLinearHeading((new Vector2d(-49.5, -38.5)), Math.toRadians(-90));
                .strafeToLinearHeading((new Vector2d(-50, -45)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(-135))
                .strafeToLinearHeading((new Vector2d(-59, -45)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(-135))
                .strafeToLinearHeading(new Vector2d(-59, -45), Math.toRadians(115))
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(-135))
                .strafeToLinearHeading(new Vector2d(-52.5, -52.5), Math.toRadians(-135))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math. toRadians(-135))
                .splineToLinearHeading(new Pose2d(-24,0, Math.toRadians(0)), Math.toRadians(0))*/




        RoadRunnerBotEntity myFifthBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        /*myFifthBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -63.5, Math.toRadians(90)))
                .lineToY(-34)
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(30,-41), Math.toRadians(90))
                .splineTo(new Vector2d(42.5,-12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(42.5,-60), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(42.5,-12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(52.5,-12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(52.5,-60), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(52.5,-12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(61.75,-12), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(61.75,-60), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40,-61), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40,-61), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40,-61), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-35), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(40,-61), Math.toRadians(90))
                .build());*/

        RoadRunnerBotEntity mySixthBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySixthBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8.5, -63, Math.toRadians(90)))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(7.25,-33.5), Math.toRadians(90))
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
                .lineToY(-63)
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(23,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(11.75,-33.5), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(15, -42, Math.toRadians(90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(33, -52, Math.toRadians(90)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(36.5,-63), Math.toRadians(-90))
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(15.5,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(9.5,-33.5), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(14, -42, Math.toRadians(90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(33, -52, Math.toRadians(90)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(36.5,-63), Math.toRadians(-90))
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(14,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5,-33.5), Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(13, -42, Math.toRadians(90)), Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(33, -52, Math.toRadians(90)), Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(36.5,-63), Math.toRadians(-90))
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(13,-44), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(2.75,-33.5), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(0,-33), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFourthBot)
                //.addEntity(mySixthBot)
                .start();


    }
}