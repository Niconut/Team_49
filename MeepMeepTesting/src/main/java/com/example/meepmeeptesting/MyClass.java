package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, 62, Math.toRadians(90)))
                .setReversed(true)
                        .lineToY(33)
                .strafeToLinearHeading(new Vector2d(49,56), Math.toRadians(-90))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(56,56), Math.toRadians(-135))
               .setReversed(false)
                .strafeToLinearHeading(new Vector2d(59,56), Math.toRadians(-90))
                .setReversed(true)
               .strafeToLinearHeading(new Vector2d(56,56), Math.toRadians(-135))
               .setReversed(false)
               // .splineTo(new Vector2d(56,56), Math.toRadians(45))
                //.setReversed(false)
                //        .splineTo(new Vector2d(56, 52), Math.toRadians(-180))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-49,56), Math.toRadians(-90))
                .setReversed(false)
                .strafeTo(new Vector2d(-59,56))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

    }
}