package frc.robot.subsystems.arm;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(ArmIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
