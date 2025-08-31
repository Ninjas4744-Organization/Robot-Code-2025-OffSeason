package frc.robot.subsystems.intakealigner;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class IntakeAlignerIOController implements IntakeAlignerIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kIntakeAlignerControllerConstants);
    }

    @Override
    public Controller getController() {
        return controller;
    }

    @Override
    public void updateInputs(IntakeAlignerIOInputsAutoLogged inputs) {
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