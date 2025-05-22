package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs extends Controller.ControllerIOInputs {
    }

    default void setAngle(Rotation2d angle) {
    }

    default void setPercent(double percent) {
    }

    default void updateInputs(ArmIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
