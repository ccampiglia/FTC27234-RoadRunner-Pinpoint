package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTestingBasket {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(-24, -63, Math.toRadians(90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                ///**** Move forward and spline behind first block and push back to OZ
                .strafeToLinearHeading(new Vector2d(-47, -52), Math.toRadians(225.00))
                .strafeToLinearHeading(new Vector2d(-60, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-47, -52), Math.toRadians(225.00))
                .strafeToLinearHeading(new Vector2d(-49, -40), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-47, -52), Math.toRadians(225.00))

                /// Back to OZ
                .strafeToLinearHeading(new Vector2d(57, -57), Math.toRadians(90.00))





                .build()

        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}