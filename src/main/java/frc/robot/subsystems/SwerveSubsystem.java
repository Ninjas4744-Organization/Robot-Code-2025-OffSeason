package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    private static SwerveSubsystem instance;
    private boolean enabled;

    public static void createInstance(SwerveSubsystem swerveSubsystem) {
        instance = swerveSubsystem;
    }

    public static SwerveSubsystem getInstance() {
        return instance;
    }

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(Constants.kSwerveConstants));
            SwerveController.setInstance(new SwerveController(Constants.kSwerveControllerConstants));
            SwerveController.getInstance().setChannel("Driver");
        }
    }

    public void swerveDrive(CommandPS5Controller controller) {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
            new SwerveInput(-MathUtil.applyDeadband(controller.getLeftY(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                -MathUtil.applyDeadband(controller.getLeftX(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                -MathUtil.applyDeadband(controller.getRightX(), Constants.kJoystickDeadband) * Constants.kDriverRotationSpeedFactor,
                Constants.kDriverFieldRelative
            )), "Driver");
    }

    @Override
    public void periodic() {
        if (enabled)
            SwerveController.getInstance().periodic();
    }
}
