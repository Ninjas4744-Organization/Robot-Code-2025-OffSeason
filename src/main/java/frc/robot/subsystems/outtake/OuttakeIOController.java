package frc.robot.subsystems.outtake;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class OuttakeIOController implements OuttakeIO{
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kOuttakeControllerConstants);
    }

    @Override
    public Controller getController() {
        return controller;
    }

    @Override
    public void updateInputs(OuttakeIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }
}
