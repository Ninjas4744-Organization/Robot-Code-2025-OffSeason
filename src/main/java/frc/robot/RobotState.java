package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

public class RobotState extends RobotStateWithSwerve<States> {
    public RobotState(SwerveDriveKinematics kinematics, boolean gyroInverted, int pigeonID, boolean enableOdometryThread) {
        super(kinematics, gyroInverted, pigeonID, enableOdometryThread);

        robotState = States.IDLE;
        CANBus = new CANBus(Constants.kCANBusName);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateWithSwerve.getInstance();
    }
}
