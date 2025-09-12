package frc.robot.subsystems.intakeangle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class IntakeAngleIOController implements IntakeAngleIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kIntakeAngleControllerConstants);
    }

    @Override
    public void updateInputs(IntakeAngleIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
        inputs.AtGoal = controller.atGoal();
    }

    @Override
    public void periodic() {
        controller.periodic();
    }

    @Override
    public void setPercent(double percent) {
        controller.setPercent(percent);
    }

    @Override
    public void setPosition(double position) {
        controller.setPosition(position);
    }
}