package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class DistanceCalc {
    private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    //private VisionPortal visionPortal;

    //private AprilTagProcessor apriTag;

    //DcMotorEx intakeMotor;
    //DcMotorEx launchMotor;
    //double launchPower;

    //shooter velocity constants
    static double g = 9.8; //m/s^2
    static double x;
    static double Theta = 89.98;
    static double HGoal = 0.984;//in meters
    static double HShoot = 0.248;//in meters

    static double Vtip;

   static double Radius = 0.0048;

   static double PulsePerRev = 28;

   static double RPM;

    static double TargetTicksPerSecond;

   static double effiencyFactor = 0.2;
    static double VelOfShooter;
   static double numerator;
   static double denominator;
    public static double DistanceCalc(){
        x=AprilTagDetection.telemetryAprilTag();
        if (x == -1) {
            VelOfShooter = 0;
        } else {
            numerator = g * Math.pow(x,2);
            denominator = 2 * Math.pow(Math.cos(Theta),2) * (x * Math.tan(Theta)-(HGoal-HShoot));
            VelOfShooter = Math.sqrt(numerator/denominator);
            RPM = (60* effiencyFactor)/(2*Math.PI*Radius);
            Vtip = RPM * (2*Math.PI*Radius)/60;
            TargetTicksPerSecond = RPM * (PulsePerRev/60);
        return(TargetTicksPerSecond);
        }
        return(-1);

    }

}
