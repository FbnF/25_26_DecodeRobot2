package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.ServoLogger;

@Disabled
@Autonomous(name="FeedServoRampLogger", group="Tuning")
public class FeedServoRampLogger extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo feedServo = hardwareMap.get(Servo.class, "feedServo");
        ServoLogger logger = new ServoLogger(feedServo);
        logger.init();

        telemetry.addLine("Ready to log servo ramp");
        telemetry.update();
        waitForStart();

        long t0 = System.currentTimeMillis();

        // sweep forward
        for (double pos = 0.0; opModeIsActive() && pos <= 1.0; pos += 0.05) {
            feedServo.setPosition(pos);
            sleep(400);
            long now = System.currentTimeMillis() - t0;
            logger.logCsv(now, pos);
        }

        // sweep backward
        for (double pos = 1.0; opModeIsActive() && pos >= 0.0; pos -= 0.05) {
            feedServo.setPosition(pos);
            sleep(400);
            long now = System.currentTimeMillis() - t0;
            logger.logCsv(now, pos);
        }

        // flush the file once at shutdown
        String stopMsg = logger.stopRobot();
        telemetry.addData("Stop", stopMsg);
        telemetry.update();

        sleep(3000); // gives time to read telemetry
    }
}