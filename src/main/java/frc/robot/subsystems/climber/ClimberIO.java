package frc.robot.subsystems.climber;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.subsystems.climber.ClimberIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
