package frc.robot;

public final class Constants {

    public static final class ElevatorConstants {
        // Motor IDs
        public static final int leaderMotor = 21;
        public static final int followerMotor = 20;

        // Limit switch port
        public static final int limitSwitchPort = 0;

        // PID coefficients
        public static final double ElevatorkP = 0.1;
        public static final double ElevatorkI = 0.0;
        public static final double ElevatorkD = 0.0;

        // Feedforward for elevator hold
        public static final double feedForward = 0.05;

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
