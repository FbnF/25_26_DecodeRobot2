package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "ServoTest")
public class ServoTest extends LinearOpMode {

    private Servo Servo1;
    private Servo Servo2;
    private Servo Servo3;
    private Servo LoadServo;


    @Override
    public void runOpMode(){

        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        Servo2 = hardwareMap.get(Servo.class, "Servo2");
        Servo3 = hardwareMap.get(Servo.class, "Servo3");
        LoadServo = hardwareMap.get(Servo.class, "LoadServo");

        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.a){
                Servo1.setPosition(0.25);
            }
            if(gamepad1.b){
                Servo2.setPosition(0.25);
            }
            if(gamepad1.y){
                Servo3.setPosition(0.25);
            }
            if(gamepad1.x){
                LoadServo.setPosition(0.5);
            }
        }
    }
}


    /** Initialize AprilTag detection pipeline **/
