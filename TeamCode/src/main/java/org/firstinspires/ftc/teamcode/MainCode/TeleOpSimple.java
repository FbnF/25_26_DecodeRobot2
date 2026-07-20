package org.firstinspires.ftc.teamcode.MainCode;

// --- Roadrunner Libraries ---
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;

// --- FTC Libraries ---
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.VoltageSensor;

// -- Defined by us ---
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.MainCode.util.Calculations;
import org.firstinspires.ftc.teamcode.MainCode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.MainCode.config.TagConfig;
import org.firstinspires.ftc.teamcode.MainCode.vision.AprilTagService;

// --- Data Logging ---
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLoggerFlex;

@TeleOp(name = "TeleOp: Simple", group = "TeleOp")
public class TeleOpSimple extends LinearOpMode {

    double speedFactor = 0.7;
    private DcMotorEx intakeMotor;
    private CRServo intakeServo1;
    private CRServo intakeServo2;

    @Override
    public void runOpMode() {


        intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2.setDirection(CRServo.Direction.REVERSE);

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.a) speedFactor = 0.95;
            if (gamepad1.b) speedFactor = 0.4;
            if (gamepad1.x) speedFactor = 0.7;
            double axial = -gamepad1.right_stick_y * speedFactor; // up = forward (+x)
            double lateral = -gamepad1.left_stick_x * speedFactor; // right = strafe right (−y)
            double heading = -gamepad1.right_stick_x * speedFactor; // right = turn right (−CCW = CW)
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), heading));
            if (gamepad2.right_trigger > 0) intakeMotor.setPower(1.0);
            if (gamepad2.right_bumper) intakeMotor.setPower(0.5);
            if (gamepad2.left_trigger > 0) intakeMotor.setPower(-1.0);
            if (gamepad2.left_bumper) intakeMotor.setPower(-0.5);
            if (gamepad2.x) intakeMotor.setPower(0.0);
            if (gamepad2.a){
                 intakeServo1.setPower(1.0);
                intakeServo2.setPower(1.0);
            }
            if (gamepad2.b){
                intakeServo1.setPower(-1.0);
                intakeServo2.setPower(-1.0);
            }
        }
    }
}