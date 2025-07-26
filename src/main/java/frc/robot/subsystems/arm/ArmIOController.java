package frc.robot.subsystems.arm;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class ArmIOController implements ArmIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kArmControllerConstants);
    }

    @Override
    public Controller getController() {
        return controller;
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }
}