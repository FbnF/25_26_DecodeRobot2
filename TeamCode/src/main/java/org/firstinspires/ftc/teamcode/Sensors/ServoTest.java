package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Sensor: Servo", group = "Sensor")
public class ServoTest extends LinearOpMode {

    private Servo test_servo;

    @Override
    public void runOpMode() {
        test_servo = hardwareMap.get(Servo.class, "test_servo");
        test_servo.setPosition(0); // Initial position

        waitForStart();

        // Run continuously until stopped
        while (opModeIsActive()) {

            // Move from 0.0 to 1.0
            for (int i = 0; i <= 10; i++) {
                double pos = i / 10.0;
                test_servo.setPosition(pos);
                sleep(500);
            }

            // Move from 1.0 back to 0.0
            for (int i = 10; i >= 0; i--) {
                double pos = i / 10.0;
                test_servo.setPosition(pos);
                sleep(500);
            }
        }
    }
}