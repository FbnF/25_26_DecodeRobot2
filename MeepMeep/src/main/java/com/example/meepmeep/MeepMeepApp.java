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
        Pose2d startPose = new Pose2d(-48, -48, Math.toRadians(225));
        // Old code: always spline to (0, 0, 225)
        Pose2d splineTarget = new Pose2d(0, 0, Math.toRadians(225));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(
                myBot.getDrive().actionBuilder(startPose)
                        // First spline and shoot
                        .setTangent(Math.toRadians(225))


                        .strafeTo(new Vector2d(-20, -20))

                        // Shooter runs


                        .splineToLinearHeading(new Pose2d(-12, -24,Math.toRadians(270)),Math.toRadians(270))

                        .lineToY(-48)
                        .lineToY(-45)

                        .strafeToLinearHeading(new Vector2d(-20, -20), Math.toRadians(225))

                        // Shooter runs
                        //Clear
                        .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(0, -54), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(0, -20), Math.toRadians(180))

                        .splineToLinearHeading(new Pose2d(12, -24,Math.toRadians(270)),Math.toRadians(270))
                        .lineToY(-52)
                        .lineToY(-45)


                        .strafeToLinearHeading(new Vector2d(-20,-20), Math.toRadians(225))




                        //Collect
                        .splineToLinearHeading(new Pose2d(20, -50,Math.toRadians(270)), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(20, -60), Math.toRadians(270))
                        //Shoot
                        .strafeToLinearHeading(new Vector2d(-20,-20), Math.toRadians(225))
                        .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(0, -54), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(0, -40), Math.toRadians(180))
                        //Collect
                        .splineToLinearHeading(new Pose2d(20, -50,Math.toRadians(270)), Math.toRadians(270))
                        .strafeToLinearHeading(new Vector2d(20, -60), Math.toRadians(270))
                        //Shoot
                        .strafeToLinearHeading(new Vector2d(-36, -12), Math.toRadians(250))

                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}