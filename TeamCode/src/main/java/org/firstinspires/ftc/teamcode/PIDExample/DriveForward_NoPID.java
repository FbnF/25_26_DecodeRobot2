package org.firstinspires.ftc.teamcode.PIDExample;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "DriveForward_NoPID", group = "PID Example")
public class DriveForward_NoPID extends LinearOpMode {

    DcMotor leftFront, leftRear, rightFront, rightRear;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRearMotor");

        // If +power drives your right side backward, uncomment these two lines:
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
        int targetTicks = 3000;
        double drivePower = 0.8;
        waitForStart();

        if (!opModeIsActive()) return;

        setAllPower(drivePower, drivePower, drivePower, drivePower);
        while (opModeIsActive() && avgTicks() < targetTicks) {
            telemetry.addData("AvgTicks", avgTicks());
            telemetry.addData("LF/LR", "%d / %d", leftFront.getCurrentPosition(), leftRear.getCurrentPosition());
            telemetry.addData("RF/RR", "%d / %d", rightFront.getCurrentPosition(), rightRear.getCurrentPosition());
            telemetry.update();
        }
        setAllPower(0, 0, 0, 0);
        sleep(400);
        telemetry.addData("Final AvgTicks (expect overshoot)", avgTicks());
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

}
