package frc.robot.coraldetection;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectionIO {
    @AutoLog
    class CoralDetectionIOInputs {
        boolean hasTarget;
        Rotation2d yaw;
    }

    default void updateInputs(CoralDetectionIOInputsAutoLogged inputs) {
    }
}
