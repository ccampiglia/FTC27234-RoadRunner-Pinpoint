package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(24, -63, Math.toRadians(90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                //**** Move forward and spline behind first block and push back to OZ
                .splineToLinearHeading(new Pose2d(37.0,-40.0, Math.toRadians(270)),Math.toRadians(90))
                .lineToY(00.0)
                .splineToConstantHeading(new Vector2d(48.00, 00.00), Math.toRadians(270))
                .lineToY(-60)

                //**** Move straight forward then spline behind second block
                //.lineToLinearHeading(new Pose2d(40.0,-5.0, Math.toRadians(270)))
                //.splineToConstantHeading(new Vector2d(54.00, 0.00), Math.toRadians(270))
                //.lineTo(new Vector2d(56.0, -50))

                //**** Move straight forward then spline behind third block
                //.lineTo(new Vector2d(56.0, 0.0))
                //.splineToConstantHeading(new Vector2d(64.00, 0.00), Math.toRadians(270))
                //.lineTo(new Vector2d(64.0, -50.0))
                //.waitSeconds(2)

                //Back up and Move to Submersible
                .lineToY(-50)
                .strafeToLinearHeading(new Vector2d(5, -37), Math.toRadians(90.00))

                //Move back to OZ
                .strafeToLinearHeading(new Vector2d(48, -50), Math.toRadians(270.00))

                //Place First Specimen
                //.lineToLinearHeading(new Pose2d(00.0,-32.0, Math.toRadians(90)))
                //.splineToLinearHeading(new Pose2d(00.0,-32.0, Math.toRadians(270)),Math.toRadians(270))
                //.waitSeconds(2)

                //Return
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}