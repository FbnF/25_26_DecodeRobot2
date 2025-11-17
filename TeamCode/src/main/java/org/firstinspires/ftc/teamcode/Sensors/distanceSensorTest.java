package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Sensor: Distance Sensor", group = "Sensor")
public class distanceSensorTest extends LinearOpMode{

    DistanceSensor sensor_distance;
    double currentDistance;

    @Override
    public void runOpMode() {
        sensor_distance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        waitForStart();

        while (opModeIsActive()) {
            currentDistance = sensor_distance.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance (cm)", currentDistance);
            telemetry.update();
        }
    }

}
