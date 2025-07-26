package frc.robot.subsystems.elevator;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
