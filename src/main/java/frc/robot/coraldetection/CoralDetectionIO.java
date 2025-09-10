package frc.robot.coraldetection;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CoralDetectionIO {
    @AutoLog
    class CoralDetectionIOInputs {
        boolean hasTargets;
        Rotation2d yaw;
    }

    default void updateInputs(CoralDetectionIOInputsAutoLogged inputs) {
    }
}
