package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d angle = Rotation2d.kZero;
    }

    public default void updateInputs(ArmIOInputs inputs) {
    }

    public default void setAngle(Rotation2d angle) {
    }
}
