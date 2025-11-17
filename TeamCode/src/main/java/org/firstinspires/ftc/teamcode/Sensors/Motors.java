package org.firstinspires.ftc.teamcode.Sensors;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Sensor: Motor", group = "Sensor")

public class Motors extends LinearOpMode {
    DcMotor motor;
    double power = 0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(power);
            if(gamepad1.a){
                motor.setPower(0.5);
            }
        }

    }

}
