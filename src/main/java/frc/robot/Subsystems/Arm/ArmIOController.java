package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class ArmIOController implements ArmIO {
    private final Controller controller;

    public ArmIOController() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kArmControllerConstants);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        controller.setPosition(angle.getRadians());
    }

    @Override
    public void setPercent(double percent) {
        controller.setPercent(percent);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }
}