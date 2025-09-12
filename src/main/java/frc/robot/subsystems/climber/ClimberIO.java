package frc.robot.subsystems.climber;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void updateInputs(ClimberIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
