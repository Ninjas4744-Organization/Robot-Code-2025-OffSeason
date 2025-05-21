package frc.robot;

public class Constants {
    public static enum RobotMode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final RobotMode kSimMode = RobotMode.SIM;
    public static final RobotMode kCurrentMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public enum ArmPositions {
        Close(0),
        Open(180),
        Mid(90);

        double angle;

        ArmPositions(double angle) {
            this.angle = angle;
        }

        public double get() {
            return angle;
        }
    }
}
