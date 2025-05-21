package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.TalonFXController;
import frc.lib.NinjasLib.dataclasses.MainControllerConstants;

public class ArmIOReal implements ArmIO {
    private final Controller controller;

    public ArmIOReal() {
        MainControllerConstants constants = new MainControllerConstants();
        constants.main.id = 20;
        constants.currentLimit = 70;
        controller = new TalonFXController(constants);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        controller.setPosition(angle.getRadians());
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angle = Rotation2d.fromRadians(controller.getPosition());
    }
}