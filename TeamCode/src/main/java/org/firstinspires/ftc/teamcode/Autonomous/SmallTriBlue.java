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

import org.firstinspires.ftc.teamcode.MainCode.util.AutoMotorControl;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="SmallTriBlue", group="Auto")
public class SmallTriBlue extends LinearOpMode {

    // RC config names
    private static final String FEED_SERVO   = "feedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    //private static final String VOLTAGE_SENSOR = "VoltageSensor";

    // Tunables
    private static final double INTAKE_POWER  = 0.73;
    private static double SHOOTER_POWER = 0.74;

    private static double SHOOTER_VEL = 1760;
    private static double SHOOTER_VEL_R2 = 1770;

    private static double SHOOTER_VEL_SEC = 1760;

    // positions (use what worked in your tests)
    private static final double SERVO_LOAD_POS = 0.0;
    private static final double SERVO_FEED_POS = 0.12;

    private static final double ANGLE_OF_TURN = -24;

    // Feed schedule at the stop (seconds from start of the shooter action)
    private static final double[] FEED_START_S = { 2, 4};
    private static final double[] FEED_START_S_FIRST = {1};

    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0;

    private static final double MAX_VOLTAGE = 12.5;

    double CURRENT_VOLTAGE = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(60, -16, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake        = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter     = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        Servo feed            = hardwareMap.get(Servo.class, FEED_SERVO);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        // VoltageSensor battery = hardwareMap.get(VoltageSensor.class, VOLTAGE_SENSOR); // read battery

        // Safe defaults
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf_cur = new PIDFCoefficients(500, 3, 0, 4);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_cur);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        // proportional compensation: keep motor voltage constant
       /* double vbat = (battery != null) ? battery.getVoltage() : 12.0;
        if (!Double.isFinite(vbat) || vbat <= 0) vbat = 12.0;
        SHOOTER_POWER = Math.min(1.0, SHOOTER_POWER * (12.0 / vbat));*/

        telemetry.addData("SHOOTER_POWER", SHOOTER_POWER);
        telemetry.update();


        waitForStart();
        telemetry.addData("SHOOTER_VELOCITY", shooter.getVelocity());
        telemetry.update();
        if (isStopRequested()) return;

        Action all = drive.actionBuilder(startPose)
                .stopAndAdd(setMotorPower(intake, 0.0))
                .strafeToLinearHeading(new Vector2d(52, -8), Math.toRadians(202))

                // .turn(Math.toRadians(-ANGLE_OF_TURN))

                //.strafeToLinearHeading(new Vector2d(60, -16), Math.toRadians(180))
                .setTangent(Math.toRadians(204))
                //.splineTo(new Vector2d(32, -36), Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(30, -26), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .lineToY(-48)
                .lineToY(-55)
                // .lineToY(-36)
                .stopAndAdd(setMotorPower(intake, 0.0))

                .strafeToLinearHeading(new Vector2d(52, -12), Math.toRadians(200))

                //.turn(Math.toRadians(ANGLE_OF_TURN))
                .setTangent(Math.toRadians(200))
                .strafeToLinearHeading(new Vector2d(11, -19), Math.toRadians(275))
                .setTangent(Math.toRadians(275))
                .lineToY(-55)
                .build();

        Actions.runBlocking(all);

        // Safety park
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);
    }
}