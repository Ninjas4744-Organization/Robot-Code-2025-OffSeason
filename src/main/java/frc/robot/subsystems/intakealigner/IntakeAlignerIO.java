package frc.robot.subsystems.intakealigner;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAlignerIO {
    @AutoLog
    class IntakeAlignerIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(IntakeAlignerIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
