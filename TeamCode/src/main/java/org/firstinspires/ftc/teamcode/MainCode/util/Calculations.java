package org.firstinspires.ftc.teamcode.MainCode.util;

/**
 * Shooter/ballistics and FTC-friendly unit helpers.
 *
 * Angles are in RADIANS unless otherwise noted.
 * Distances are in METERS unless otherwise noted.
 */
public final class Calculations {

    private Calculations() {} // static-only class

    // --- Unit helpers ---
    public static double inchesToMeters(double inches) { return inches * 0.0254; }
    public static double metersToInches(double meters) { return meters / 0.0254; }
    public static double degreesToRadians(double deg) { return Math.toRadians(deg); }
    public static double radiansToDegrees(double rad) { return Math.toDegrees(rad); }

    // --- Core physics helpers ---

    /**
     * Required exit (linear) velocity to hit a target at horizontal distance x, given a fixed launch angle.
     *
     * v^2 = (g x^2) / (2 cos^2θ * (x tanθ - Δh)x), where Δh = shooterH - targetH
     *
     * @param g          gravity (m/s^2), e.g. 9.81
     * @param x          horizontal distance to target (m) (>= 0)
     * @param launchRad  launch angle above horizontal (radians)
     * @param shooterH   shooter center height (m)
     * @param targetH    target center height (m)
     * @return required linear exit velocity (m/s). Returns NaN if geometry is impossible.
     */
    public static double requiredExitVelocity(double g, double x, double launchRad,
                                              double shooterH, double targetH) {
        final double cos = Math.cos(launchRad);
        final double tan = Math.tan(launchRad);
        final double deltaH = targetH - shooterH;
        final double denom = 2.0 * cos * cos * (x * tan - deltaH);

        if (x <= 0 || denom <= 0 || g <= 0) return Double.NaN;

        return Math.sqrt((g * x * x) / denom);
    }

    /**
     * Convert linear exit velocity to wheel RPM, assuming a direct “surface speed” relationship.
     *
     * v = ω r  =>  ω = v / r ;  RPM = ω * 60 / (2π)
     *
     * @param exitVelMS      linear exit velocity needed (m/s)
     * @param wheelRadiusM   shooter wheel radius (m)
     * @param efficiency     0 < efficiency ≤ 1, lumped losses/slip (lower → needs higher RPM)
     * @return wheel surface RPM (double.NaN if invalid inputs)
     */
    public static double exitVelocityToWheelRPM(double exitVelMS, double wheelRadiusM, double efficiency) {
        if (exitVelMS <= 0 || wheelRadiusM <= 0 || efficiency <= 0) return Double.NaN;
        final double omega = exitVelMS / (wheelRadiusM * efficiency); // rad/s
        return (omega * 60.0) / (2.0 * Math.PI);
    }

    /**
     * Convert wheel RPM to motor ticks per second for DcMotorEx.setVelocity().
     */
    public static double rpmToTicksPerSecond(double rpm, double ticksPerRev) {
        if (rpm < 0 || ticksPerRev <= 0) return Double.NaN;
        return (rpm * ticksPerRev) / 60.0;
    }

    // --- Convenience one-shot pipeline ---

    /**
     * Full pipeline: inches → meters → physics → RPM → ticks/s.
     *
     * @return ticks per second for DcMotorEx.setVelocity(), or NaN if impossible
     */
    public static double computeTPSFromRangeInches(double g,
                                                   double horizontalInches,
                                                   double launchDeg,
                                                   double shooterHeightM,
                                                   double targetHeightM,
                                                   double wheelRadiusM,
                                                   double efficiency,
                                                   double ticksPerRev) {
        final double xMeters = inchesToMeters(horizontalInches);
        final double theta = degreesToRadians(launchDeg);

        double vExit = requiredExitVelocity(g, xMeters, theta, shooterHeightM, targetHeightM);
        if (Double.isNaN(vExit)) return Double.NaN;

        double rpm = exitVelocityToWheelRPM(vExit, wheelRadiusM, efficiency);
        if (Double.isNaN(rpm)) return Double.NaN;

        return rpmToTicksPerSecond(rpm, ticksPerRev);
    }
}