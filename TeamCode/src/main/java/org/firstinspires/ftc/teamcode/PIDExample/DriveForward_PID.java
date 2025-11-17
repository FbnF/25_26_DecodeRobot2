package org.firstinspires.ftc.teamcode.PIDExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name = "DriveForward_PID", group = "PID Example")
public class DriveForward_PID extends LinearOpMode {

    DcMotor leftFront, leftRear, rightFront, rightRear;

    // --- PID gains (tune on field) ---
    double kP = 0.0008;     // power per tick of error
    double kI = 0.00010;    // power per (tick * sec) accumulated
    double kD = 0.00030;    // power per tick/sec (damping)

    // --- Limits & behavior ---
    double outMax   = 0.60;   // clamp motor power magnitude
    int    stopBand = 20;     // stop when |error| <= this many ticks
    long   timeoutMs = 8000;  // safety timeout

    // Integral anti-windup / conditional integration
    double iMaxContribution = 0.30;    // max |kI * integral| added to output
    int    integrateWhenErrorBelow = 500; // only build I when close to target
    double integralBleed = 0.98;       // bleed I when not integrating

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // If +power drives your right side backward, uncomment:
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int targetTicks = 3000; // same target as your NoPID example

        waitForStart();
        if (!opModeIsActive()) return;

        // --- PID state ---
        ElapsedTime timer = new ElapsedTime();
        double lastTime = timer.seconds();
        int    lastError = targetTicks - avgTicks();
        double integral  = 0.0;

        long t0 = System.currentTimeMillis();

        while (opModeIsActive()) {
            // Timing
            double now = timer.seconds();
            double dt  = now - lastTime;
            if (dt <= 0) dt = 1e-3;  // guard against zero/neg dt
            lastTime = now;

            // Measurement & error
            int pos   = avgTicks();
            int error = targetTicks - pos;

            // Derivative (on error)
            double dError = (error - lastError) / dt;
            lastError = error;

            // Conditional integration & anti-windup
            boolean allowI = Math.abs(error) <= integrateWhenErrorBelow && kI != 0.0;
            if (allowI) {
                integral += error * dt;  // accumulate error*time
                // Clamp integral so kI*integral stays within ±iMaxContribution
                double maxInt = (iMaxContribution == 0) ? 0 : (iMaxContribution / Math.max(Math.abs(kI), 1e-9));
                if (integral >  maxInt) integral =  maxInt;
                if (integral < -maxInt) integral = -maxInt;
            } else {
                integral *= integralBleed; // gentle decay when not integrating
            }

            // Terms
            double pTerm = kP * error;
            double iTerm = kI * integral;
            double dTerm = kD * dError;

            // Output (clamped)
            double out = pTerm + iTerm + dTerm;
            out = clip(out, -outMax, outMax);

            // Straight drive (same power both sides)
            setAllPower(out, out, out, out);

            // Telemetry for learning/tuning
            telemetry.addData("AvgTicks", pos);
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Err", error);
            telemetry.addData("P/I/D", "%.3f / %.3f / %.3f", pTerm, iTerm, dTerm);
            telemetry.addData("Out", out);
            telemetry.addData("dt(ms)", Math.round(dt * 1000));
            telemetry.update();

            // End conditions
            if (Math.abs(error) <= stopBand) break;
            if (System.currentTimeMillis() - t0 > timeoutMs) break;
        }

        setAllPower(0, 0, 0, 0);
        sleep(400);
        telemetry.addData("Final AvgTicks (PID)", avgTicks());
        telemetry.update();
        sleep(6000);
    }

    private int avgTicks() {
        return (leftFront.getCurrentPosition()
                + leftRear.getCurrentPosition()
                + rightFront.getCurrentPosition()
                + rightRear.getCurrentPosition()) / 4;
    }

    private void setAllPower(double lf, double lr, double rf, double rr) {
        leftFront.setPower(lf); leftRear.setPower(lr); rightFront.setPower(rf); rightRear.setPower(rr);
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}