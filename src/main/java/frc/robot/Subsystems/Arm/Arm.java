package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    //    private final ArmIO io;
    private Controller controller;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;
    private boolean enabled;

    public static Arm getInstance() {
        return instance;
    }

    public static void createInstance(boolean enabled) {
        instance = new Arm(enabled);
    }

    public Arm(boolean enabled) {
        if (enabled) {
            //        io = new ArmIOController();
            controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kArmControllerConstants);
        }
        this.enabled = enabled;
    }

    public void setAngle(Rotation2d angle) {
        if (enabled) {
            //        io.setAngle(angle);
            controller.setPosition(angle.getRadians());
        }
    }

    public void setPercent(double percent) {
        if (enabled) {
            //        io.setPercent(percent);
            controller.setPercent(percent);
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

//        io.periodic();
        controller.periodic();

//        io.updateInputs(inputs);
        controller.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }
}