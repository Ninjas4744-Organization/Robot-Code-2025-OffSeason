package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d angle = Rotation2d.kZero;
    }

    /**
     * Update the set of loggable inputs.
     */
    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setAngle(Rotation2d angle) {
    }
}
