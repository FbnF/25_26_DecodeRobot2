package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
@Disabled
@TeleOp(name = "Sensor: Color sensor", group = "Sensor")
public class SensorColor extends LinearOpMode {

  NormalizedColorSensor myColorSens;

  @Override
  public void runOpMode() {
    myColorSens = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

    waitForStart();

    while (opModeIsActive()) {
      NormalizedRGBA c = myColorSens.getNormalizedColors();
      telemetry.addData("R", "%.3f", c.red);
      telemetry.addData("G", "%.3f", c.green);
      telemetry.addData("B", "%.3f", c.blue);
      telemetry.addData("Alpha", "%.3f", c.alpha);
      telemetry.update();

    }
  }
}
