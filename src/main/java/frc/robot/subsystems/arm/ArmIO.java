package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs extends Controller.ControllerIOInputs {
        Rotation2d AbsoluteAngle;
    }

    default void setup() {
    }

    default Rotation2d getCANCoder(){
        return null;
    }

    default Controller getController() {
        return null;
    }

    default void updateInputs(ArmIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
