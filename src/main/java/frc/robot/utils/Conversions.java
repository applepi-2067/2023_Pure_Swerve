package frc.robot.utils;


public class Conversions {
    public static double RPMToTicksPer100ms(double RPM, double ticksPerRev) {
        return RPM * ticksPerRev / (60.0 * 10.0);
    }

    public static double ticksPer100msToRPM(double ticksPer100ms, double ticksPerRev) {
        return ticksPer100ms * (10.0 * 60.0) * (1.0 / ticksPerRev);
    }


    public static double ticksToMeters(double ticks, double ticksPerRev, double gearRatio, double radiusMeters) {
        return ticks * (1.0 / ticksPerRev) * (1.0 / gearRatio) * (Math.PI * 2.0 * radiusMeters);
    }

    public static double metersToTicks(double meters, double ticksPerRev, double gearRatio, double radiusMeters) {
        return meters * (1.0 / (Math.PI * 2.0 * radiusMeters)) * ticksPerRev * gearRatio;
    }


    public static double metersPerSecondToRPM(double metersPerSecond, double radiusMeters) {
        return metersPerSecond * 60.0 / (radiusMeters  * (2.0 * Math.PI));
    }

    public static double rpmToMetersPerSecond(double RPM, double radiusMeters) {
        return RPM * (1.0 / 60.0) * (2 * Math.PI) * radiusMeters;
    }
}
