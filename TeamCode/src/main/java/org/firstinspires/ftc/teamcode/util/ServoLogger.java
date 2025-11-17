package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class ServoLogger {
    private final Servo servo;
    private final StringBuffer m_csvLogString = new StringBuffer();

    public ServoLogger(Servo servo) {
        this.servo = servo;
    }

    public void init() {
        m_csvLogString.setLength(0);
        m_csvLogString.append("time_ms,commandedPos,measuredPos\n");
    }

    public void logCsv(double timeMs, double commanded) {
        double measured = servo.getPosition();
        m_csvLogString.append(
                String.format(Locale.US, "%.0f,%.3f,%.3f\n", timeMs, commanded, measured)
        );
    }

    public String stopRobot() {
        String result = "Stop complete";
        try {
            File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/data");
            if (!dir.exists()) dir.mkdirs();

            File out = new File(dir, "feedServo_ramp.csv");
            try (FileWriter fw = new FileWriter(out)) {
                fw.write(m_csvLogString.toString());
            }
        } catch (IOException e) {
            result = e.getMessage();
        }
        return result;
    }
}