package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    //    private final ElevatorIO io;
    private Controller controller;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;
    private boolean enabled;

    public static Elevator getInstance() {
        return instance;
    }

    public static void createInstance(boolean enabled) {
        instance = new Elevator(enabled);
    }

    private Elevator(boolean enabled) {
        if (enabled) {
            //        io = new ElevatorIOController();
            controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.kElevatorControllerConstants);
        }
        this.enabled = enabled;
    }

    public void setHeight(double height) {
        if (enabled) {
            //        io.setAngle(angle);
            controller.setPosition(height);
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
        Logger.processInputs("Elevator", inputs);
    }
}