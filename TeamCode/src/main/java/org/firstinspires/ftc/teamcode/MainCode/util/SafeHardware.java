package org.firstinspires.ftc.teamcode.MainCode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public final class SafeHardware {
    private SafeHardware() {}

    public static DcMotor tryDcMotor(HardwareMap hw, String name) {
        try { return hw.get(DcMotor.class, name); } catch (Exception e) { return null; }
    }

    public static DcMotorEx tryDcMotorEx(HardwareMap hw, String name) {
        try { return hw.get(DcMotorEx.class, name); } catch (Exception e) { return null; }
    }

    public static Servo tryServo(HardwareMap hw, String name) {
        try { return hw.get(Servo.class, name); } catch (Exception e) { return null; }
    }
}