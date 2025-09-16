package frc.robot.coraldetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class CoralDetection {
    private CoralDetectionIO io;
    private final CoralDetectionIOInputsAutoLogged inputs = new CoralDetectionIOInputsAutoLogged();

    public CoralDetection(CoralDetectionIO io) {
        this.io = io;
    }

    public void periodic() {
        if(Robot.isSimulation())
            return;

        io.updateInputs(inputs);
        Logger.processInputs("Coral Detection", inputs);
    }

    public boolean hasTargets() {
        return inputs.hasTargets;
    }

    public Rotation2d getYaw() {
        return inputs.yaw;
    }

    public Translation2d getFieldRelativeDir() {
        return new Translation2d(1, Swerve.getInstance().getGyro().getYaw().rotateBy(inputs.yaw.unaryMinus()));
    }
}