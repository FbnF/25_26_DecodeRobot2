package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket; // ADDED
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// ADDED
import org.firstinspires.ftc.teamcode.MainCode.util.TinyCsvLogger;
@Disabled
@Autonomous(name="MEET1: SmallTriRedNoTurn", group="MainAuto")
public class AutoSmallTriangleSimpleBlueSideNoTurn extends LinearOpMode {
    // ---- Hardware names ----
    private static final String FEED_SERVO   = "FeedServo";
    private static final String INTAKE_MOTOR = "IntakeMotor";
    private static final String LAUNCH_MOTOR = "LaunchMotor";

    // ---- Tunables ----
    private static final double INTAKE_POWER  = 0.60;  // runs while base moves
    private static final double SHOOTER_POWER = 0.74;  // open-loop; swap to velocity if you want

    // Servo positions (use your tested mid-range)
    private static final double SERVO_LOAD_POS = -5.00;
    private static final double SERVO_FEED_POS = 0.75;

    // Three feed windows while shooter is spinning (seconds from action start)
    private static final double[] FEED_START_S = {1.0, 3.0, 5.0};
    private static final double   FEED_HOLD_S  = 0.7;
    private static final double   END_PADDING_S = 1.0; // extra LOAD time after last feed

    /** One-shot action to set motor power (non-blocking; completes immediately). */
    private static Action setMotorPower(DcMotor m, double p) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(p);
            }
            return false; // RR Actions: false = finished (do not block)
        };
    }

    /**
     * Combined shooter+feeder action.
     * return TRUE to keep running, FALSE when complete.
     * While this returns TRUE, the follower is paused because we insert it with .stopAndAdd(...).
     */
    private static class ShooterAndFeederAction implements Action {
        private final DcMotorEx shooter;
        private final Servo feeder;
        private final double shooterPower;
        private final double[] starts;
        private final double holdS;
        private final double endPadS;

        private boolean inited = false;
        private long t0;

        ShooterAndFeederAction(DcMotorEx shooter,
                               Servo feeder,
                               double shooterPower,
                               double[] starts, double holdS, double endPadS) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.shooterPower = shooterPower;
            this.starts = starts;
            this.holdS = holdS;
            this.endPadS = endPadS;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!inited) {
                t0 = System.nanoTime();
                if (shooter != null) {
                    shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooter.setPower(shooterPower);
                }
                feeder.setPosition(SERVO_LOAD_POS);
                inited = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            boolean feeding = false;
            for (double s : starts) {
                if (t >= s && t < s + holdS) { feeding = true; break; }
            }
            feeder.setPosition(feeding ? SERVO_FEED_POS : SERVO_LOAD_POS);

            packet.put("t_s", String.format("%.2f", t));
            packet.put("feeding", feeding);

            double lastEnd = starts[starts.length - 1] + holdS + endPadS;

            if (t < lastEnd) {
                return false;   // keep running (base stays paused)
            }

            // Finish: park servo, stop shooter
            feeder.setPosition(SERVO_LOAD_POS);
            if (shooter != null) shooter.setPower(0.0);
            return false;      // done
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Drive + hardware
        Pose2d startPose = new Pose2d(60, 10, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor intake = hardwareMap.get(DcMotor.class, INTAKE_MOTOR);
        DcMotorEx shooter = (DcMotorEx) hardwareMap.get(DcMotor.class, LAUNCH_MOTOR);
        Servo feed = hardwareMap.get(Servo.class, FEED_SERVO);
        // feed.setDirection(Servo.Direction.REVERSE); // if your linkage is inverted

        // ADDED: DcMotorEx handle for intake (for logging only)
        DcMotorEx intakeExForLog = hardwareMap.get(DcMotorEx.class, INTAKE_MOTOR);

        // ADDED: create CSV logger for Auto
        TinyCsvLogger logger = TinyCsvLogger.create(hardwareMap, "auto_smalltri_blue_noturn");

        // Default safe states
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
        shooter.setPower(0.0);
        feed.setPosition(SERVO_LOAD_POS);

        waitForStart();
        if (isStopRequested()) {
            try { logger.close(); } catch (Exception ignored) {}
            return;
        }

        Action routine = drive.actionBuilder(startPose)
                .setTangent(0)
                .stopAndAdd(setMotorPower(shooter, 0.74))
                .lineToX(startPose.position.x+1)
                .lineToX(startPose.position.x)
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S))
                .waitSeconds(1)
                .lineToX(startPose.position.x+1)
                .lineToX(startPose.position.x)
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S))
                .waitSeconds(1)
                .lineToX(startPose.position.x+1)
                .lineToX(startPose.position.x)
                .stopAndAdd(new ShooterAndFeederAction(
                        shooter, feed, SHOOTER_POWER,
                        FEED_START_S, FEED_HOLD_S, END_PADDING_S))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d( -20, -20, Math.toRadians(-225)), Math.PI / 2)
                .build();

        // ADDED: wrap the routine with a per-tick logger
        Action logged = new Action() {
            @Override
            public boolean run(TelemetryPacket packet) {
                // advance odometry
                drive.updatePoseEstimate();

                // read pose + powers
                Pose2d pose = drive.localizer.getPose();
                double launchCmd = shooter.getPower(); // last-set power as "command" in Auto
                double intakeCmd = intake.getPower();

                // write one CSV row
                logger.record(
                        "run",
                        launchCmd,
                        shooter,
                        intakeCmd,
                        intakeExForLog,
                        feed,
                        pose
                );

                // continue original routine
                return routine.run(packet);
            }
        };

        // run the logged action
        Actions.runBlocking(logged);

        // Safety
        feed.setPosition(SERVO_LOAD_POS);
        shooter.setPower(0.0);
        intake.setPower(0.0);

        // ADDED: close the logger
        logger.close();
    }

}