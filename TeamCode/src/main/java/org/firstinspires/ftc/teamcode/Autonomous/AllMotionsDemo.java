package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "RR v1.0 - All Motions Demo (Corrected Tangents)", group = "RoadRunner")
public class AllMotionsDemo extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Start at origin, heading = 0 rad (east)
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build one continuous action so pose/tangent carry correctly between segments.
        Action all = drive.actionBuilder(startPose)
                //new code for square


                // ---- X motion & turning ----
              .lineToX( 20)
                //.lineToX(0)
                .turn(Math.toRadians(90))
                .lineToX( 20)


                // 0 -> +90
                //.turn(Math.toRadians(-180))   // +90 -> -90
                .turn(Math.toRadians(180))// -90 -> 0
                .lineToY( 20)
                .turn(Math.toRadians(270))// -90 -> 0
                .lineToX( 20)
                .turn(Math.toRadians(360))// -90 -> 0
                .lineToY( 20)
                .splineTo(new Vector2d(24, 24), Math.toRadians(0))  // end at (24,24), heading 0
/*


                // ---- Y motion (needs Y tangent) ----
                .setTangent(Math.toRadians(90))   // travel along +Y
                .lineToY(24)
                .lineToY(0)
                .setTangent(Math.toRadians(0))    // back to +X travel

                // ---- splineTo out & back (same heading at end of first) ----
                .splineTo(new Vector2d(24, 24), Math.toRadians(0))  // end at (24,24), heading 0
                .setTangent(Math.toRadians(225))                     // from (24,24) to (0,0) is SW
                .splineTo(new Vector2d(0, 0), Math.toRadians(225))
                .setTangent(Math.toRadians(0))

                // ---- constant-heading splines (keep heading; follow path tangent) ----
                .setTangent(Math.toRadians(270))                                        // go “down” (−Y)
                .splineToConstantHeading(new Vector2d(24, -24), Math.toRadians(270))
                .setTangent(Math.toRadians(135))                                        // from (24,-24) to (0,0) is NW
                .splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(135))
                .setTangent(Math.toRadians(0))

                // ---- linear-heading splines (interpolate heading) ----
                // approach angles use atan2(dy, dx) for clean geometry
                .splineToLinearHeading(
                        new Pose2d(-24, -12, Math.toRadians(90)),
                        Math.toRadians(206.565)   // direction from (0,0) -> (-24,-12)
                )
                .splineToLinearHeading(
                        new Pose2d(0, 0, Math.toRadians(0)),
                        Math.toRadians(26.565)    // direction from (-24,-12) -> (0,0)
                )

                // ---- spline-heading splines (smooth pose + path tangent) ----
                .splineToSplineHeading(
                        new Pose2d(-24, 12, Math.toRadians(45)),
                        Math.toRadians(153.435)   // direction from (0,0) -> (-24,12)
                )
                .splineToSplineHeading(
                        new Pose2d(0, 0, Math.toRadians(0)),
                        Math.toRadians(333.435)   // direction from (-24,12) -> (0,0)
                )
                */

                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);
    }
}