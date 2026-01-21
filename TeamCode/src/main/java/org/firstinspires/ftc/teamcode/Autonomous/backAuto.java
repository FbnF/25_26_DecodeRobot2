package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sun.tools.javac.util.MandatoryWarningHandler;

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="backAuto", group="Auto")
public class backAuto extends LinearOpMode {

    //private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                .turn(Math.toRadians(90))
                .build();

        Actions.runBlocking(all);

        // Safety park
    }
}