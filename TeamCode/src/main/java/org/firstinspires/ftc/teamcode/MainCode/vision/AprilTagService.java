package org.firstinspires.ftc.teamcode.MainCode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;

import java.util.List;

public class AprilTagService {
    private VisionPortal portal;
    private AprilTagProcessor processor;

    // Smoothing state
    private double lastDistanceIn = Double.NaN;

    public static class Reading {
        public final boolean hasTag;
        public final int id;
        public final double xIn, yIn, zIn;      // inches
        public final double rangeIn;            // inches (straight-line)
        public final double bearingDeg, elevDeg;
        public final double smoothedDistanceIn; // inches, based on TagConfig.USE_RANGE/Y and SMOOTH_ALPHA

        public Reading(boolean hasTag, int id,
                       double xIn, double yIn, double zIn,
                       double rangeIn, double bearingDeg, double elevDeg,
                       double smoothedDistanceIn) {
            this.hasTag = hasTag;
            this.id = id;
            this.xIn = xIn;
            this.yIn = yIn;
            this.zIn = zIn;
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.elevDeg = elevDeg;
            this.smoothedDistanceIn = smoothedDistanceIn;
        }
    }

    /** Build processor + portal. Call once in init. */
    public void start(HardwareMap hardwareMap) {
        // ensure clean restart if start() is called again
        stop();

        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(TagConfig.DRAW_AXES)
                .setDrawTagOutline(TagConfig.DRAW_OUTLINE)
                .setDrawTagID(TagConfig.DRAW_ID)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (TagConfig.USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, TagConfig.WEBCAM_NAME));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(processor);
        portal = builder.build();
    }

    /** Stop and free camera resources (e.g., at OpMode end). */
    public void stop() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
        processor = null;
        lastDistanceIn = Double.NaN;
    }

    public Reading getLatest() {
        if (processor == null) {
            return new Reading(false, -1, 0, 0, 0, 0, 0, 0, Double.NaN);
        }

        List<AprilTagDetection> dets = processor.getDetections();
        if (dets == null || dets.isEmpty()) {
            lastDistanceIn = smoothValue(Double.NaN, lastDistanceIn, TagConfig.SMOOTH_ALPHA);
            return new Reading(false, -1, 0, 0, 0, 0, 0, 0, lastDistanceIn);
        }

        AprilTagDetection best = pickBest(dets);
        if (best == null || best.ftcPose == null) {
            lastDistanceIn = smoothValue(Double.NaN, lastDistanceIn, TagConfig.SMOOTH_ALPHA);
            return new Reading(false, -1, 0, 0, 0, 0, 0, 0, lastDistanceIn);
        }

        double measuredDistanceIn = TagConfig.USE_RANGE ? best.ftcPose.range : best.ftcPose.y;

        lastDistanceIn = smoothValue(measuredDistanceIn, lastDistanceIn, TagConfig.SMOOTH_ALPHA);

        return new Reading(
                true,
                best.id,
                best.ftcPose.x,
                best.ftcPose.y,
                best.ftcPose.z,
                best.ftcPose.range,
                best.ftcPose.bearing,
                best.ftcPose.elevation,
                lastDistanceIn
        );
    }

    private AprilTagDetection pickBest(List<AprilTagDetection> dets) {
        AprilTagDetection preferred = null;
        AprilTagDetection closest = null;
        double bestRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : dets) {
            if (d == null || d.ftcPose == null) continue;

            if (TagConfig.PREFERRED_ID > 0 && d.id == TagConfig.PREFERRED_ID) {
                preferred = d;
            }
            if (d.ftcPose.range < bestRange) {
                bestRange = d.ftcPose.range;
                closest = d;
            }
        }
        return (preferred != null) ? preferred : closest;
    }

    private static double smoothValue(double newVal, double prev, double alpha) {
        // alpha in (0,1], where 1 means no smoothing
        if (Double.isNaN(newVal)) return prev;
        if (Double.isNaN(prev)) return newVal;
        alpha = clamp(alpha, 0.0, 1.0);
        return alpha * newVal + (1.0 - alpha) * prev;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}