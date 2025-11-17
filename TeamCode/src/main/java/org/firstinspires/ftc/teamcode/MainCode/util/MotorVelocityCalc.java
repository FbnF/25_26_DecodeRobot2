package org.firstinspires.ftc.teamcode.MainCode.util;

public class MotorVelocityCalc {
    //shooter velocity constants
    public static double g = 9.8; //m/s^2

    public double Theta = 46 * Math.PI / 180;
    public static double HGoal = 0.984;//in meters
    public double HShoot = 0.248;//in meters
    public double denominator;
    public double numerator;
    public double efficiencyFactor = 0.30;
    public double VelOfShooter;
    //  double Vtip;
    public double Radius = 0.048;
    public static double PulsePerRev = 28;
    public double RPM;
    public double TargetTicksPerSecond = 0;
    public double Vtip;
    public static double ChargedBattery = 12.5;

    public double CompensatedVelocity;


    public double getVelocity(double x, double currentVoltage){
        //calculates TargetTps used to set velocity of the motor
        x = x * 0.0254; //inches to meter conversion
        numerator = g * Math.pow(x, 2);
        denominator = 2 * Math.pow(Math.cos(Theta), 2) * (x * Math.tan(Theta) - (HGoal - HShoot));
        VelOfShooter = Math.sqrt(numerator / denominator);
        RPM = (60 * VelOfShooter) / (2 * Math.PI * Radius * efficiencyFactor);
        // Vtip = RPM * (2 * Math.PI * Radius) / 60;
        TargetTicksPerSecond = RPM * (PulsePerRev / 60);

        //Calculates for variation in volts

        CompensatedVelocity = TargetTicksPerSecond - ((currentVoltage - ChargedBattery)*140);
        CompensatedVelocity = Math.min(2800, CompensatedVelocity);
        return CompensatedVelocity;


    }

}
