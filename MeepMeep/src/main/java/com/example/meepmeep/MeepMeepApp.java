package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepApp {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // FTC autonomous start pose
        Pose2d startPose = new Pose2d(60, -16, Math.toRadians(180));
        // Old code: always spline to (0, 0, 225)
        Pose2d splineTarget = new Pose2d(0, 0, Math.toRadians(225));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        // First spline and shoot
                        .splineToLinearHeading(splineTarget, Math.PI / 2)
                        .turn(Math.toRadians(45))
                        // First artifact round
                        .setTangent(0)
                        .lineToX(-10)
                        .setTangent(Math.PI / 2)
                        .strafeTo(new Vector2d(-10, -52))
                        // Second spline and shoot
                        .setTangent(0)
                        .splineToLinearHeading(splineTarget, Math.PI / 2)
                        .turn(Math.toRadians(45))
                        .setTangent(0)
                        .lineToX(10)
                        .setTangent(Math.PI / 2)
                        .strafeTo(new Vector2d(10, -52))
                        // Third spline and shoot
                        .setTangent(0)
                        .splineToLinearHeading(splineTarget, Math.PI / 2)
                        .turn(Math.toRadians(45))
                        .setTangent(0)
                        .lineToX(34)
                        .setTangent(Math.PI / 2)
                        .strafeTo(new Vector2d(34, -52))
                        // Fourth/final spline and shoot
                        .setTangent(0)
                        .splineToLinearHeading(splineTarget, Math.PI / 2)
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}