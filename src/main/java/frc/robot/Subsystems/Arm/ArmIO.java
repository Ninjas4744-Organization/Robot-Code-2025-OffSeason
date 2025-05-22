package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs extends Controller.ControllerIOInputs {

    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setAngle(Rotation2d angle) {
    }

    public default void setPercent(double percent) {
    }

    public default void periodic() {
    }
}
