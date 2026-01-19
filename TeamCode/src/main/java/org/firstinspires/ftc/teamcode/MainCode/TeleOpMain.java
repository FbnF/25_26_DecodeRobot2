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

@TeleOp(name = "TeleOp: Main", group = "TeleOp")
public class TeleOpMain extends LinearOpMode {

    // --- Hardware ---
    private Servo feedServo;
    private Servo hoodServo;
    private MecanumDrive drive;
    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;
    private DcMotorEx launchMotor2;
    private DcMotorEx TurretMotor;
    private RevBlinkinLedDriver blinkin; // LED
    private VoltageSensor battery;

    // --- Vision ---
    private AprilTagService tagService;
    private boolean visionEnabled = false; // allows camera to be toggled on/off

    // Auto shooter (closed-loop velocity) path
    private boolean autoShooter = false;
    private boolean prevDpadUp = false, prevDpadDown = false;
    private double shooterSetpointTPS = 0.0;
    private static final double NO_SETPOINT = 0.0;

    // --- Config flags ---
    private static final boolean LOG_ENABLED = true;  // turn CSV logging on/off
    private TinyCsvLoggerFlex logger; // logging Data
    private static final int GOAL_TAG_ID = 20;        // 20 = blue goal, 24 = red goal

    // require driver to arm auto-spin before controlling flywheel
    private boolean autoSpinArmed = false;
    private boolean prevDpadRight = false;

    // flash window when Y pressed too soon
    private long yTooSoonFlashUntilNs = 0L;
    private static final long FLASH_YELLOW_NS = 500_000_000L; // 500 ms

    // --- Drive/settings ---
    private double speedFactor = 0.7;
    final double SPEED_MIN = 0.2;
    final double SPEED_MAX = 1.0;
    final double SPEED_STEP = 0.1;
    boolean drivePrevRB = false, drivePrevLB = false;

    // --- Intake/servo state ---
    private double intakePower = 0.0;
    private static double launchPower;
    private boolean isIntakeRunning = false;
    private boolean isLaunchRunning = false;
    private boolean isFeedServoDown = false;

    // --- Button edge detection ---
    private boolean prevRB = false;

    private boolean feedPulseActive = false;
    private long feedPulseStartNs = 0;
    private static final long FEED_DWELL_NS = 150_000_000L; // 150 ms

    // edge state for GP1 dpad-down (vision toggle)
    private boolean prevG1DpadDown = false;

    // ---------------- HOOD SERVO FIX ----------------
    // Persisted hood position + edge detection so it steps once per press
    private double hoodServoPos = 0.0;
    private static final double HOOD_STEP = 0.05;
    private boolean prevG1DpadLeft = false;
    private boolean prevG1DpadRight = false;

    @Override
    public void runOpMode() {

        // Map hardware
        feedServo    = hardwareMap.get(Servo.class, "feedServo");
        hoodServo    = hardwareMap.get(Servo.class, "hoodServo");
        intakeMotor  = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        launchMotor  = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        launchMotor2 = hardwareMap.get(DcMotorEx.class, "LaunchMotor2");
        TurretMotor = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        battery      = hardwareMap.voltageSensor.iterator().next();

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        // Initial positions
        feedServo.setPosition(0.0);
        isFeedServoDown = false;

        hoodServoPos = 0.0;                 // start hood at 0.0
        hoodServo.setPosition(hoodServoPos);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // launchMotor2.setDirection(DcMotor.Direction.REVERSE);

        // intake runs open-loop (no encoder feedback)
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Drive (verify your constructor signature)
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Vision
        tagService = new AprilTagService();
        // tagService.start(hardwareMap);

        waitForStart();

        // Safe startup
        intakeMotor.setPower(0.0);
        launchMotor.setPower(0.0);
        launchMotor2.setPower(0.0);

        while (opModeIsActive()) {

            // # # # Gamepad 1 (Driver) # # #

            // -------------------------------- Base Drive -----------------------------------------
            if (gamepad1.a) speedFactor = 0.95;
            if (gamepad1.b) speedFactor = 0.4;
            if (gamepad1.x) speedFactor = 0.7;

            // ---------------- HOOD SERVO CONTROL (FIXED) ----------------
            // Edge detect so one press = one step, and we clamp to [0, 1]
            boolean leftEdge  = gamepad1.dpad_left  && !prevG1DpadLeft;
            boolean rightEdge = gamepad1.dpad_right && !prevG1DpadRight;

            if (leftEdge)  hoodServoPos += HOOD_STEP;
            if (rightEdge) hoodServoPos -= HOOD_STEP;

            hoodServoPos = clamp01(hoodServoPos);
            hoodServo.setPosition(hoodServoPos);

            prevG1DpadLeft  = gamepad1.dpad_left;
            prevG1DpadRight = gamepad1.dpad_right;

            double axial   = -gamepad1.right_stick_y * speedFactor; // up = forward (+x)
            double lateral = -gamepad1.left_stick_x  * speedFactor; // right = strafe right (−y)
            double heading = -gamepad1.right_stick_x * speedFactor; // right = turn right (−CCW = CW)

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(axial, lateral), heading));

            // Update odometry and read pose
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Speed Factor", "%.2f", speedFactor);

            // \--- Vision toggle (edge-based, no sleep) ---
            boolean g1DownEdge = gamepad1.dpad_down && !prevG1DpadDown;
            if (g1DownEdge) {
                if (visionEnabled) {
                    tagService.stop();
                    visionEnabled = false;
                } else {
                    tagService.start(hardwareMap);
                    visionEnabled = true;
                }
            }
            prevG1DpadDown = gamepad1.dpad_down;

            // # # # Gamepad 2 (Controls) # # #
            // --------------------------- MODE TOGGLES -------------------------
            boolean upEdge   = gamepad2.dpad_up && !prevDpadUp;
            boolean downEdge = gamepad2.dpad_down && !prevDpadDown;
            if (upEdge) {
                autoShooter = true;
                autoSpinArmed = false;
                launchMotor.setPower(0.0);
                launchMotor2.setPower(0.0);
            }
            if (downEdge) {
                autoShooter = false;
                autoSpinArmed = false;
                launchMotor.setPower(0.0);
                launchMotor2.setPower(0.0);
            }
            prevDpadUp = gamepad2.dpad_up;
            prevDpadDown = gamepad2.dpad_down;

            // Dpad-right → arm auto spin
            boolean rightEdge2 = gamepad2.dpad_right && !prevDpadRight;
            if (rightEdge2 && autoShooter) {
                autoSpinArmed = !autoSpinArmed;
            }
            prevDpadRight = gamepad2.dpad_right;

            // --------------------------- MANUAL MODE --------------------------
            if (!autoShooter) {
                if (gamepad2.a) launchPower = 0.75;
                if (gamepad2.b) launchPower = 0.60;
                if (gamepad2.left_bumper) launchPower = 0.55;
                if (gamepad2.x) launchPower = 0.0;

                // Battery compensation for open-loop power
                double vbat = battery.getVoltage();
                double scaledPower = Math.min(1.0, launchPower * (12.0 / vbat));
                launchMotor.setPower(scaledPower);
                launchMotor2.setPower(scaledPower);
            }

            // --------------------------- AUTO MODE ----------------------------
            if (autoShooter) {
                if (autoSpinArmed) {
                    Double dInches = getVisionDistanceInches();
                    if (dInches != null && dInches >= ShooterConfig.MIN_RANGE_IN) {
                        double tps = Calculations.computeTPSFromRangeInches(
                                ShooterConfig.G, dInches,
                                ShooterConfig.LAUNCH_DEG,
                                ShooterConfig.SHOOTER_H_M,
                                ShooterConfig.TARGET_H_M,
                                ShooterConfig.WHEEL_RADIUS_M,
                                ShooterConfig.EFFICIENCY,
                                ShooterConfig.TICKS_PER_REV
                        );
                        if (!Double.isNaN(tps) && Double.isFinite(tps)) {
                            if (ShooterConfig.TEST_TPS > 0) {
                                tps = ShooterConfig.TEST_TPS;
                            }
                            tps = Math.min(tps, ShooterConfig.TPS_MAX);
                            shooterSetpointTPS = tps;      // set after overrides/clamp
                            launchMotor.setVelocity(tps);  // single call
                            launchMotor2.setVelocity(tps);
                        } else {
                            shooterSetpointTPS = 0.0;
                            launchMotor.setPower(0.0);
                            launchMotor2.setPower(0.0);
                        }
                    } else {
                        shooterSetpointTPS = 0.0;
                        launchMotor.setPower(0.0);
                        launchMotor2.setPower(0.0);
                    }
                } else {
                    shooterSetpointTPS = 0.0;
                    launchMotor.setPower(0.0);
                    launchMotor2.setPower(0.0);
                }
            }

            // --------------------------- FEED LOGIC ---------------------------
            boolean spunUpOk = false;
            if (autoShooter && shooterSetpointTPS > 0.0) {
                double vel = launchMotor.getVelocity();
                spunUpOk = Math.abs(vel - shooterSetpointTPS) <= ShooterConfig.TPS_TOL;
            } else if (!autoShooter) {
                spunUpOk = (launchMotor.getPower() > 0.0);
            }

            if (gamepad2.y) {
                if (!feedPulseActive && spunUpOk) {
                    feedServo.setPosition(0.75);
                    feedPulseActive = true;
                    feedPulseStartNs = System.nanoTime();
                } else if (!spunUpOk) {
                    yTooSoonFlashUntilNs = System.nanoTime() + FLASH_YELLOW_NS;
                }
            }

            if (feedPulseActive && System.nanoTime() - feedPulseStartNs >= FEED_DWELL_NS) {
                feedServo.setPosition(0.0);
                feedPulseActive = false;
            }

            // --------------------------- INTAKE -------------------------------
            boolean rbEdge = gamepad2.right_bumper && !prevRB;
            if (rbEdge) intakePower = 0.5;
            prevRB = gamepad2.right_bumper;
            if (gamepad2.right_trigger > 0) intakePower = -1;
            if (gamepad2.left_trigger > 0) intakePower = 0.0;
            intakeMotor.setPower(intakePower);

            // --------------------------- LED STATES ---------------------------
            RevBlinkinLedDriver.BlinkinPattern pat = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            AprilTagService.Reading r = tagService.getLatest();
            boolean hasTag = (r != null && r.hasTag);
            boolean correctTag = hasTag && (r.id == GOAL_TAG_ID);

            if (!visionEnabled) {
                pat = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (!correctTag) {
                pat = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else {
                boolean atSpeed = spunUpOk && autoSpinArmed && autoShooter;
                pat = atSpeed ? RevBlinkinLedDriver.BlinkinPattern.GREEN
                        : RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            }
            if (System.nanoTime() < yTooSoonFlashUntilNs) {
                pat = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
            }
            blinkin.setPattern(pat);

            // --------------------------- LOGGING ------------------------------
            if (LOG_ENABLED && logger != null) {
                logger.record("run");
            }

            // ------------- Telemetry data -------------------------------------------------
            double tpsMeas = launchMotor.getVelocity();
            double rpmMeas = (tpsMeas * 60.0) / ShooterConfig.TICKS_PER_REV;
            Double visInches = getVisionDistanceInches();
            int tagId = (r != null && r.hasTag) ? r.id : -1;

            telemetry.addLine("---- Drive ----");
            telemetry.addData("Speed Factor", "%.2f", speedFactor);

            telemetry.addLine("---- Hood ----");
            telemetry.addData("hoodPosCmd", "%.2f", hoodServoPos);
            telemetry.addData("hoodPosReported", "%.2f", hoodServo.getPosition());

            telemetry.addLine("---- Shooter ----");
            telemetry.addData("Mode", autoShooter ? "AUTO" : "MANUAL");
            telemetry.addData("Armed", autoSpinArmed);
            telemetry.addData("Setpoint TPS", "%.0f", shooterSetpointTPS);
            telemetry.addData("Actual TPS", "%.0f", tpsMeas);
            telemetry.addData("Actual RPM", "%.0f", rpmMeas);
            if (!autoShooter) telemetry.addData("Manual Power", "%.2f", launchPower);
            telemetry.addData("Ready?", spunUpOk);

            telemetry.addLine("---- Vision ----");
            telemetry.addData("Vision Enabled", visionEnabled);
            telemetry.addData("Tag ID", tagId);
            telemetry.addData("Goal Tag ID", GOAL_TAG_ID);
            telemetry.addData("Correct Tag", correctTag);
            telemetry.addData("Range (in)", (visInches == null) ? "N/A" : String.format("%.1f", visInches));
            telemetry.addData("LED", pat.name());

            telemetry.update();
        }

        // cleanup
        try {
            launchMotor.setPower(0.0);
            intakeMotor.setPower(0.0);
        } finally {
            tagService.stop();
            if (LOG_ENABLED && logger != null) logger.close();
        }
    }

    private Double getVisionDistanceInches() {
        if (tagService == null) return null;
        AprilTagService.Reading r = tagService.getLatest();
        if (r == null || !r.hasTag) return null;
        double d = r.smoothedDistanceIn;
        if (!TagConfig.USE_RANGE && !Double.isNaN(d)) d = Math.abs(d);
        return Double.isNaN(d) ? null : d;
    }

    private static double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }
}