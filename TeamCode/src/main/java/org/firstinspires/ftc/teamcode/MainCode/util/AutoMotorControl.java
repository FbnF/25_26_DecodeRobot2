package org.firstinspires.ftc.teamcode.MainCode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Reusable Road Runner Actions:
 *  - setMotorPower(...) : one-shot, non-blocking motor power setter
 *  - ShooterAndFeederAction : spins shooter, pulses feed servo via schedule, then stops shooter
 *
 * IMPORTANT (your RR flavor): Action.run() returns TRUE to keep running, FALSE when finished.
 */
public final class AutoMotorControl {

    private AutoMotorControl() {} // no instances

    /** One-shot action that sets a motor's power and immediately completes (non-blocking). */
    public static Action setMotorPower(DcMotor m, double power) {
        return (TelemetryPacket pkt) -> {
            if (m != null) {
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                m.setPower(power);
            }
            return false; // finished immediately; does NOT block follower
        };
    }
    public static Action setServoPosition(Servo s, double pos) {
        return (TelemetryPacket pkt) -> {
            if (s != null) s.setPosition(pos);
            return false; // finished immediately
        };
    }
    public static class TimedMotorPowerAction implements Action {
        private final DcMotorEx motor;
        private final double power;
        private final double durationS;
        private final boolean stopAtEnd;

        private boolean initialized = false;
        private long t0;

        public TimedMotorPowerAction(DcMotorEx motor, double power, double durationS, boolean stopAtEnd) {
            this.motor = motor;
            this.power = power;
            this.durationS = durationS;
            this.stopAtEnd = stopAtEnd;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (motor != null) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motor.setPower(power);
                }
                initialized = true;
            }
            double t = (System.nanoTime() - t0) / 1e9;
            packet.put("timedMotor_t_s", String.format("%.2f", t));

            if (t < durationS) return true;

            if (motor != null && stopAtEnd) motor.setPower(0.0);
            return false;
        }
    }
    public static class ServoPulseAction implements Action {
        private final Servo servo;
        private final double loadPos;
        private final double feedPos;

        private final double startDelayS; // delay before first pulse
        private final double holdS;       // time spent at FEED
        private final double gapS;        // time between pulse starts (period - holdS)
        private final int repeats;        // number of pulses
        private final double endPaddingS; // extra LOAD time after last pulse

        private boolean initialized = false;
        private long t0;

        public ServoPulseAction(
                Servo servo,
                double loadPos,
                double feedPos,
                double startDelayS,
                double holdS,
                double gapS,
                int repeats,
                double endPaddingS
        ) {
            this.servo = servo;
            this.loadPos = loadPos;
            this.feedPos = feedPos;
            this.startDelayS = Math.max(0, startDelayS);
            this.holdS = Math.max(0, holdS);
            this.gapS = Math.max(0, gapS);
            this.repeats = Math.max(0, repeats);
            this.endPaddingS = Math.max(0, endPaddingS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (servo != null) servo.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;
            boolean feeding = false;

            if (t >= startDelayS && repeats > 0) {
                double tSinceStart = t - startDelayS;
                double period = Math.max(holdS + gapS, holdS); // ensure sane period
                // Which pulse index are we in?
                int idx = (int) Math.floor(tSinceStart / period);
                if (idx < repeats) {
                    double tIntoPulse = tSinceStart - idx * period;
                    feeding = (tIntoPulse >= 0 && tIntoPulse < holdS);
                }
            }

            if (servo != null) servo.setPosition(feeding ? feedPos : loadPos);

            // Telemetry
            packet.put("servoPulse_t_s", String.format("%.2f", t));
            packet.put("servoPulse_feeding", feeding);

            // Compute absolute end time
            double totalActive = startDelayS + (repeats > 0 ? (repeats * (holdS + gapS)) - gapS : 0); // last pulse doesn't need trailing full gap to "exist"
            double lastEnd = totalActive + endPaddingS;

            if (t < lastEnd) return true;

            if (servo != null) servo.setPosition(loadPos);
            return false;
        }
    }
    public static class ServoScheduleAction implements Action {
        private final Servo servo;
        private final double loadPos;
        private final double feedPos;
        private final double[] feedStartS;
        private final double feedHoldS;
        private final double endPaddingS;

        private boolean initialized = false;
        private long t0;



        public ServoScheduleAction(
                Servo servo,
                double loadPos,
                double feedPos,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS
        ) {
            this.servo = servo;
            this.loadPos = loadPos;
            this.feedPos = feedPos;
            this.feedStartS = (feedStartS != null) ? feedStartS : new double[0];
            this.feedHoldS = Math.max(0, feedHoldS);
            this.endPaddingS = Math.max(0, endPaddingS);
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (servo != null) servo.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) {
                    feeding = true;
                    break;
                }
            }
            if (servo != null) servo.setPosition(feeding ? feedPos : loadPos);

            packet.put("servoSched_t_s", String.format("%.2f", t));
            packet.put("servoSched_feeding", feeding);

            double lastEnd = (feedStartS.length > 0 ? feedStartS[feedStartS.length - 1] : 0.0)
                    + feedHoldS + endPaddingS;

            if (t < lastEnd) return true;

            if (servo != null) servo.setPosition(loadPos);
            return false;
        }
    }


    public static class ShooterAndFeederAction implements Action {
        private final DcMotorEx shooter;
        private final Servo feeder;
        private final double shooterPower;
        private final double[] feedStartS;
        private final double feedHoldS;
        private final double endPaddingS;
        private final double loadPos;
        private final double feedPos;
        private double Vcurrent;

        private VoltageSensor battery;



        private boolean initialized = false;
        private long t0;

        public ShooterAndFeederAction(
                DcMotorEx shooter,
                Servo feeder,
                double shooterPower,
                double[] feedStartS,
                double feedHoldS,
                double endPaddingS,
                double loadPos,
                double feedPos

        ) {
            this.shooter = shooter;
            this.feeder = feeder;
            this.shooterPower = shooterPower;
            this.feedStartS = (feedStartS != null) ? feedStartS : new double[0];
            this.feedHoldS = feedHoldS;
            this.endPaddingS = endPaddingS;
            this.loadPos = loadPos;
            this.feedPos = feedPos;
          //  battery = hardwareMap.voltageSensor.iterator().next();
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            if (!initialized) {
                t0 = System.nanoTime();
                if (shooter != null) {

                   // shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                  //  Vcurrent = battery.getVoltage();
              //      double shooterPowerAdapted = Math.min(1.0, shooterPower + (Vcurrent - VMax) * 0.05);
                  //  double shooterPowerAdapted = shooterPower * VMax/Vcurrent;
                    shooter.setPower(shooterPower);
                }
                if (feeder != null) feeder.setPosition(loadPos);
                initialized = true;
            }

            double t = (System.nanoTime() - t0) / 1e9;

            // Inside any FEED window?
            boolean feeding = false;
            for (double s : feedStartS) {
                if (t >= s && t < s + feedHoldS) { feeding = true; break; }
            }
            if (feeder != null) feeder.setPosition(feeding ? feedPos  : loadPos);

            // Telemetry (optional dashboard insight)
            packet.put("t_s", String.format("%.2f", t));
            packet.put("feeding", feeding);

            double lastEnd = (feedStartS.length > 0 ? feedStartS[feedStartS.length - 1] : 0.0)
                    + feedHoldS + endPaddingS;

            // TRUE = keep running, FALSE = done (per your RR build)
            if (t < lastEnd) return true;

            // Finish
            if (feeder != null) feeder.setPosition(loadPos);
            if (shooter != null) shooter.setPower(0.0);
            return false;
        }
    }
}