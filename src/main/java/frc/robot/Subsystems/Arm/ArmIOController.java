package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.Controllers.NinjasController;
import frc.lib.NinjasLib.Controllers.NinjasTalonFXController;
import frc.lib.NinjasLib.DataClasses.MainControllerConstants;

public class ArmIOController implements ArmIO {
    private final NinjasController controller;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public ArmIOController() {
        controller = new NinjasTalonFXController(new MainControllerConstants());
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angle = Rotation2d.fromDegrees(controller.getPosition());
    }

    @Override
    public void setVoltage(double volts) {
        roller.setControl(voltageRequest.withOutput(volts));
    }
}