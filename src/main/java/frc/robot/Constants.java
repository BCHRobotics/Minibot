package frc.robot;

public final class Constants {

    public static final class ElevatorConstants {
        // Motor IDs
        public static final int leaderMotor = 21;
        public static final int followerMotor = 20;

        // Limit switch port
        public static final int topLimitSwitchPort = 1 ;
        public static final int bottomLimitSwitchPort = 0;

        // Position limits in inches
        public static final double bottomPos = 0.0;
        public static final double L1 = 12.0;
        public static final double L2 = 24.0;
        public static final double L3 = 36.0;
        public static final double topPos = 36.0;

        // Encoder counts per inch
        public static final double countsPerInch = 1024.0 / 12.0;

        // Maximum motor output
        public static final double maxOutput = 1.0;
    }
}
