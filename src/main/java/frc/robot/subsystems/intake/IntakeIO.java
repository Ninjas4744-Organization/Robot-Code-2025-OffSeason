package frc.robot.subsystems.intake;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(IntakeIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
