package frc.robot.subsystems.outtake;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    class OuttakeIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(OuttakeIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
