package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.NinjasLib.MathUtils;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

public class RobotState extends RobotStateWithSwerve<States> {
    private static final DigitalInput intakeBeamBreaker = new DigitalInput(5);
    private int L = 1;

    public int getL() {
        return L;
    }

    public void setL(int L) {
        this.L = MathUtils.clamp(L,1,4);
    }

    public static boolean isCoralInIntake() {
        return intakeBeamBreaker.get();
    }

    public RobotState(SwerveDriveKinematics kinematics, boolean gyroInverted, int pigeonID, boolean enableOdometryThread) {
        super(kinematics, gyroInverted, pigeonID, enableOdometryThread);

        robotState = States.IDLE;
        CANBus = new CANBus(Constants.kCANBusName);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateWithSwerve.getInstance();
    }
}
