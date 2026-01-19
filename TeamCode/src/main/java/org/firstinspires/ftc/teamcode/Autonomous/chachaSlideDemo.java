package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "chacha", group = "RoadRunner")
public class chachaSlideDemo extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Start at origin, heading = 0 rad (east)
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build one continuous action so pose/tangent carry correctly between segments.
        Action all = drive.actionBuilder(startPose)
                // ---- X motion & turning ----
                .setTangent(Math.toRadians(0))
                .lineToX(30)
                .lineToX(0)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .lineToY(-34)
                .lineToY(-29)
                .lineToY(-34)
                .lineToY(-29)
                .lineToY(-34)
                .lineToY(-29)
                .lineToY(-34)
                .lineToY(-29)
                .lineToY(-34)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToX(-36)
                .lineToX(0)
                .turn(Math.toRadians(360))
                .turn(Math.toRadians(360))
                .turn(Math.toRadians(360))
                .setTangent(Math.toRadians(90))
                .lineToY(0)
                .lineToY(-34)
                // ---- splineTo out & back (same heading at end of first) ----\

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);
    }
}