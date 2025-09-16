package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.MathUtils;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

public class RobotState extends RobotStateWithSwerve<States> {
    private static int L = 1;

    public static RobotState getInstance() {
        return (RobotState) RobotStateWithSwerve.getInstance();
    }

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);

        robotState = States.IDLE;
        CANBus = new CANBus(Constants.kCANBusName);
    }

    public static int getL() {
        return L;
    }

    public static void setL(int L) {
        RobotState.L = MathUtils.clamp(L, 1, 4);
    }
}
