package frc.robot.subsystems.intake_angle;

import frc.lib.NinjasLib.controllers.Controller;
//import frc.robot.subsystems.intake.IntakeAngleIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAngleIO {
    @AutoLog
    class IntakeAngleIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(IntakeAngleIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
