package frc.robot.Constants;

import frc.robot.Robot;

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

    public static final RobotMode simMode = RobotMode.SIM;
    public static final RobotMode currentMode = Robot.isReal() ? RobotMode.REAL : simMode;
}
