package org.firstinspires.ftc.teamcode.MainCode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/** Ultra-minimal CSV logger for TeleOp/Auto metrics. */
public final class TinyCsvLogger {
    private final BufferedWriter bw;
    private final VoltageSensor vSensor;
    private final long t0Ns;
    private final String urlHint;

    private TinyCsvLogger(BufferedWriter bw, VoltageSensor vSensor, String urlHint) {
        this.bw = bw;
        this.vSensor = vSensor;
        this.t0Ns = System.nanoTime();
        this.urlHint = urlHint;
    }

    /** Create a logger writing to FIRST/data/teleop_<stamp>.csv and emit a header. */
    public static TinyCsvLogger create(HardwareMap hw, String runTag) {
        try {
            File dir = AppUtil.ROBOT_DATA_DIR; // FIRST/data
            if (!dir.exists()) dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            String fname = (runTag != null ? runTag : "log") + "_" + stamp + ".csv";
            File out = new File(dir, fname);
            BufferedWriter bw = new BufferedWriter(new FileWriter(out));
            // header
            bw.write(
                    "t_ms,tag,batt_V," +
                            "launch_cmd,launch_power,launch_tps," +
                            "intake_cmd,intake_power,intake_tps,intake_has_enc," +
                            "feed_pos,x_m,y_m,head_rad\n"
            );
            bw.flush();

            // pick any available voltage sensor
            VoltageSensor vs = null;
            for (VoltageSensor v : hw.getAll(VoltageSensor.class)) { vs = v; break; }

            String url = "http://192.168.43.1:8080/db/FIRST/data/" + fname;
            return new TinyCsvLogger(bw, vs, url + (runTag == null ? "" : "  tag=" + runTag));
        } catch (Exception e) {
            throw new RuntimeException("TinyCsvLogger init failed: " + e.getMessage(), e);
        }
    }

    /** Record one line. Keep calls cheap to avoid impacting loop timing. */
    public void record(String tag,
                       double launchCmd,
                       DcMotorEx launchMotor,
                       double intakeCmd,
                       DcMotorEx intakeMotor,
                       Servo feedServo,
                       Pose2d pose) {
        try {
            long t_ms = (System.nanoTime() - t0Ns) / 1_000_000L;
            double batt = (vSensor != null) ? vSensor.getVoltage() : Double.NaN;

            // Launcher measured
            double lmPower = (launchMotor != null) ? launchMotor.getPower()    : Double.NaN; // last set power
            double lmTps   = (launchMotor != null) ? launchMotor.getVelocity() : Double.NaN; // ticks/sec

            // Intake measured (+ encoder flag if mode supports it)
            boolean intakeHasEnc = false;
            double imPower = (intakeMotor != null) ? intakeMotor.getPower() : Double.NaN;
            double imTps   = Double.NaN;
            if (intakeMotor != null) {
                try {
                    DcMotor.RunMode mode = intakeMotor.getMode();
                    intakeHasEnc = (mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (intakeHasEnc) {
                        imTps = intakeMotor.getVelocity();
                    }
                } catch (Exception ignored) { /* leave defaults */ }
            }

            // Servo (commanded position only — no feedback available)
            double feedPos = (feedServo != null) ? feedServo.getPosition() : Double.NaN;

            // Pose (Road Runner 1.0)
            double x    = (pose != null) ? pose.position.x         : Double.NaN;
            double y    = (pose != null) ? pose.position.y         : Double.NaN;
            double head = (pose != null) ? pose.heading.toDouble() : Double.NaN;

            // tag is free-form; avoid commas to keep CSV simple
            String safeTag = (tag == null) ? "" : tag.replace(",", " ");

            bw.write(String.format(
                    Locale.US,
                    "%d,%s," +                 // t_ms, tag
                            "%.3f," +                  // batt_V
                            "%.3f,%.3f,%.2f," +        // launch_cmd, launch_power, launch_tps
                            "%.3f,%.3f,%.2f,%d," +     // intake_cmd, intake_power, intake_tps, intake_has_enc
                            "%.3f," +                  // feed_pos
                            "%.3f,%.3f,%.4f\n",        // x_m, y_m, head_rad
                    t_ms, safeTag,
                    batt,
                    launchCmd, lmPower, lmTps,
                    intakeCmd, imPower, imTps, (intakeHasEnc ? 1 : 0),
                    feedPos,
                    x, y, head
            ));
            bw.flush(); // flush each line so partial runs still save
        } catch (Exception ignored) {
            // intentionally swallow to keep opmodes resilient
        }
    }

    /** Close file. Safe to call multiple times. */
    public void close() {
        try { bw.close(); } catch (Exception ignored) {}
    }

    /** Convenience to show the exact download URL in telemetry once. */
    public String getUrlHint() { return urlHint; }
}