package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.dataclasses.FOMCalculator;

public class RobotState extends RobotStateWithSwerve<States> {
    public RobotState(SwerveDriveKinematics kinematics, boolean gyroInverted, FOMCalculator fomCalculator, int pigeonID) {
        super(kinematics, gyroInverted, fomCalculator, pigeonID);

        robotState = States.IDLE;
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateWithSwerve.getInstance();
    }
}
