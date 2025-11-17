package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Sensor: LED Light Strip", group = "Sensor")
public class LEDLightStrip extends LinearOpMode {

    private Servo blinkin;
    double blinkinValue;
    double servoPosition;
    double step;


    @Override
    public void runOpMode(){
        blinkin = hardwareMap.get(Servo.class, "blinkin");

        double [] patternValues = {

                0.67, // Red
                0.69, // Yellow
                0.71, // Green
                0.73, // Blue
        };

        waitForStart();

        while (opModeIsActive()) {
            step = 0.02;
            for (blinkinValue = -0.99; blinkinValue <= 0.99; blinkinValue += step) {
                servoPosition = (blinkinValue + 1.0) / 2.0;
                blinkin.setPosition(servoPosition);
                telemetry.addData("Blinkin Value", blinkinValue);
                telemetry.addData("Servo Position", servoPosition);

                telemetry.update();
                sleep(3000);
            }

        }
    }
}
