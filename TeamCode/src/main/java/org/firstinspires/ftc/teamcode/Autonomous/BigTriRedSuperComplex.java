package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BigTriRedSuperComplex", group = "Auto")
public class BigTriRedSuperComplex extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Start at origin, heading = 0 rad (east)
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build one continuous action so pose/tangent carry correctly between segments.
        Action all = drive.actionBuilder(startPose)
                // First spline and shoot
                .setTangent(Math.toRadians(135))


                .strafeTo(new Vector2d(-20, 20))

                // Shooter runs


                .splineToLinearHeading(new Pose2d(-12, 24,Math.toRadians(90)),Math.toRadians(90))

                .lineToY(48)
                .lineToY(45)

                .strafeToLinearHeading(new Vector2d(-20, 20), Math.toRadians(135))

                // Shooter runs
                //Clear
                .strafeToLinearHeading(new Vector2d(0, 40), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(0, 54), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(0, 20), Math.toRadians(0))

                .splineToLinearHeading(new Pose2d(12, 24,Math.toRadians(90)),Math.toRadians(90))
                .lineToY(52)
                .lineToY(45)


                .strafeToLinearHeading(new Vector2d(-20,20), Math.toRadians(135))




                //Collect
                .splineToLinearHeading(new Pose2d(20, 50,Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(20, 60), Math.toRadians(90))
                //Shoot
                .strafeToLinearHeading(new Vector2d(-20,20), Math.toRadians(135))
                .strafeToLinearHeading(new Vector2d(0, 40), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(0, 54), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(0, 40), Math.toRadians(0))
                //Collect
                .splineToLinearHeading(new Pose2d(20, 50,Math.toRadians(90)), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(20, 60), Math.toRadians(90))
                //Shoot
                .strafeToLinearHeading(new Vector2d(-36, 12), Math.toRadians(120))

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);
    }
}