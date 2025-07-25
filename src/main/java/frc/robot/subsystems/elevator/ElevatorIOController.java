package frc.robot.subsystems.elevator;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class ElevatorIOController implements ElevatorIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kElevatorControllerConstants);
    }

    @Override
    public Controller getController() {
        return controller;
    }

    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }
}