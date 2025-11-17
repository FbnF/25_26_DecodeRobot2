package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Disabled
@TeleOp(name = "AprilTag Example", group = "Examples")
public class AprilTagTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();

        }
    }

    /** Initialize AprilTag detection pipeline **/
    private void initAprilTag() {
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

    /** Improved Telemetry Output **/
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections.isEmpty()) {
            telemetry.addLine("🟥 AprilTag: NOT DETECTED");
            telemetry.addLine("Make sure the tag is visible to the camera.");
            return;
        }

        telemetry.addLine("🟩 AprilTag: DETECTED");
        telemetry.addData("Total Tags Seen", detections.size());

        for (AprilTagDetection tag : detections) {
            // Map tag ID to name
            String tagName;
            switch (tag.id) {
                case 21: tagName = "GPP (ID 21)"; break;
                case 22: tagName = "PGP (ID 22)"; break;
                case 23: tagName = "PPG (ID 23)"; break;
                default: tagName = "Unknown (" + tag.id + ")"; break;
            }

            telemetry.addLine("------------------------------------");
            telemetry.addData("Tag", tagName);

            if (tag.metadata != null) {
                telemetry.addData("Position (in)",
                        String.format("X: %.1f  Y: %.1f  Z: %.1f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                telemetry.addData("Orientation (deg)",
                        String.format("Yaw: %.1f  Pitch: %.1f  Roll: %.1f",
                                tag.ftcPose.yaw, tag.ftcPose.pitch, tag.ftcPose.roll));
                telemetry.addData("Range/Bearing/Elev",
                        String.format("%.1f in, %.1f°, %.1f°",
                                tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
            } else {
                telemetry.addData("Tag Center (px)",
                        String.format("(%.0f, %.0f)", tag.center.x, tag.center.y));
            }
        }

        telemetry.addLine("------------------------------------");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up)");
        telemetry.addLine("Yaw = Rotation Left/Right");
    }
}