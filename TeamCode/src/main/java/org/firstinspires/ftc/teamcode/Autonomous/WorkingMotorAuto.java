package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.ShooterAndFeederAction;
import static org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl.setMotorPower;

@Disabled
@Autonomous(name="Coach_Sidd's_AutoMain", group="Main")
public class WorkingMotorAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, 22, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Motors you want to toggle during "waits"
        DcMotor intakeMotor      = hardwareMap.get(DcMotor.class, "IntakeMotor");
        DcMotorEx launchMotor    = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        Servo feedServo          = hardwareMap.get(Servo.class, "feedServo");
        VoltageSensor battery    = hardwareMap.get(VoltageSensor.class, "VoltageSensor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);
        feedServo.setPosition(0.0);

        // Tunables (nominal @ 12V)
        final double INTAKE_POWER     = 1.0;
        double SHOOTER_POWER          = 1.0; // adjust to your tuned value at 12V
        final double SERVO_LOAD_POS   = 0.00;
        final double SERVO_FEED_POS   = 0.75;

        // Shooter feed schedule (seconds from start of the shooter action)
        final double[] FEED_START_S   = {4.52, 6.52, 9.52};
        final double   FEED_HOLD_S    = 0.7;
        final double   END_PADDING_S  = 1.0;

        // Proportional battery compensation (keep effective motor voltage ~constant)
        double vbat = (battery != null) ? battery.getVoltage() : 12.0;
        if (!Double.isFinite(vbat) || vbat <= 0) vbat = 12.0;
        SHOOTER_POWER = Math.min(1.0, SHOOTER_POWER * (12.0 / vbat));

        // Create specific, reusable actions (new shooter + intake logic)
        Action shooterVolley = new ShooterAndFeederAction(
                launchMotor, feedServo,
                SHOOTER_POWER,
                FEED_START_S, FEED_HOLD_S, END_PADDING_S,
                SERVO_LOAD_POS, SERVO_FEED_POS
        );
        Action intakeOn  = setMotorPower(intakeMotor, INTAKE_POWER);
        Action intakeOff = setMotorPower(intakeMotor, 0.0);

        Action all = drive.actionBuilder(startPose)
                // --- Leg 1 ---
                .splineTo(new Vector2d(15, -10), Math.toRadians(135))

                // LAUNCH SEQUENCE: Stop, run volley with feeder, continue
                .stopAndAdd(shooterVolley)

                .setTangent(Math.toRadians(90))
                // INTAKE SEQUENCE: Start intake and run in parallel
                .afterDisp(0.0, intakeOn)
                .splineToLinearHeading(new Pose2d(-11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                // Stop intake
                .afterDisp(0.0, intakeOff)

                // --- Leg 2 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(shooterVolley)

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, intakeOn)
                .splineToLinearHeading(new Pose2d(11, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, intakeOff)

                // --- Leg 3 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(shooterVolley)

                .setTangent(Math.toRadians(90))
                .afterDisp(0.0, intakeOn)
                .splineToLinearHeading(new Pose2d(34, 33, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(45)
                .lineToY(33)
                .afterDisp(0.0, intakeOff)

                // --- Leg 4 ---
                .splineTo(new Vector2d(0, 0), Math.toRadians(-40))
                .stopAndAdd(shooterVolley)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(all);

        // safety
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);
        feedServo.setPosition(SERVO_LOAD_POS);
    }
}