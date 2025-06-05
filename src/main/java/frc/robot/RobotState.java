package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.RobotStateWithSwerve;

public class RobotState extends RobotStateWithSwerve<States> {
    public RobotState(SwerveDriveKinematics kinematics, boolean gyroInverted, int pigeonID) {
        super(kinematics, gyroInverted, pigeonID);

        robotState = States.IDLE;
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateWithSwerve.getInstance();
    }
}
