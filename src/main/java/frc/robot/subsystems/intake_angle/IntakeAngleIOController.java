package frc.robot.subsystems.intake_angle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class IntakeAngleIOController implements IntakeAngleIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kIntakeAngleControllerConstants);
    }

    @Override
    public Controller getController() {
        return controller;
    }

    @Override
    public void updateInputs(IntakeAngleIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }

    public void setPercent(double percent) {
        controller.setPercent(percent);
    }
}