package frc.robot;

/** 
 * Constants.java stores fixed values that cannot be changed after they are initilized 
 * ex. motor controller CAN IDs, controller ports 
 */
public final class Constants {


    public static class ElevatorConstants {

        public static final int leftElevatorID = 20;
        public static final int rightElevatorID = 21;

        public static final double ElevatorkP = 0.2;
        public static final double ElevatorkI = 0;
        public static final double ElevatorkD = 0;
        public static final double maxOutput = 0.6;

        public static final double countsPerInch = 10.0;

        public static final double downPos = 0.0;
        public static final double bottomPos = 0.0;
        public static final double topPos = 60.0;
        public static final double L1 = 20;
        public static final double L2 = 40;
        public static final double L3 = 60;
		public static final int limitSwitchPort = 0;

        public static final double feedForward = 0.05;

         
    }

    public static final class CONTROLLER {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

}