package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetection {
    private  static  final boolean USE_WEBCAM = true;
    private static VisionPortal visionPortal;

    public static double x;

    public static AprilTagProcessor aprilTag;


    public static double telemetryAprilTag() {
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("🟥 AprilTag: NOT DETECTED");
            telemetry.addLine("Make sure the tag is visible to the camera.");
            return (-1);
        }

        telemetry.addLine("🟩 AprilTag: DETECTED");
        telemetry.addData("Total Tags Seen", detections.size());

        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag : detections) {
            // Map tag ID to name
            String tagName;
            switch (tag.id) {
                case 21:
                    tagName = "GPP (ID 21)";
                    break;
                case 22:
                    tagName = "PGP (ID 22)";
                    break;
                case 23:
                    tagName = "PPG (ID 23)";
                    break;
                default:
                    tagName = "Unknown (" + tag.id + ")";
                    break;
            }

            telemetry.addLine("------------------------------------");
            telemetry.addData("Tag", tagName);

            if (tag.metadata != null) {
                telemetry.addData("Position (in)",
                        String.format("X: %.1f  Y: %.1f  Z: %.1f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
             //   telemetry.addData("Orientation (deg)",
               ///         String.format("Yaw: %.1f  Pitch: %.1f  Roll: %.1f",
                            //    tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll));
              //  telemetry.addData("Range/Bearing/Elev",
                //       String.format("%.1f in, %.1f°, %.1f°",
                                //tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
            } else {
                telemetry.addData("Tag Center (px)",
                      String.format("(%.0f, %.0f)", tag.center.x, tag.center.y));
            }
            return (tag.ftcPose.y * 0.0254);

        }
        return(0);
    }
    public static void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

}
