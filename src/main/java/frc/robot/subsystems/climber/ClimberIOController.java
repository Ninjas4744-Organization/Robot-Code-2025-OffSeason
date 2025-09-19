package frc.robot.subsystems.climber;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class ClimberIOController implements ClimberIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.Climber.kControllerConstants);
    }

    @Override
    public void updateInputs(ClimberIOInputsAutoLogged inputs) {
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