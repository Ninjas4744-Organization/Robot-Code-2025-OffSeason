package frc.robot.coraldetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class CoralDetection {
    private CoralDetectionIO io;
    private final CoralDetectionIOInputsAutoLogged inputs = new CoralDetectionIOInputsAutoLogged();
    private boolean enabled;

    private PhotonCamera camera;
    private boolean hasTarget;
    private Rotation2d yaw;
    private static CoralDetection instance;

    public CoralDetection(boolean enabled, CoralDetectionIO io) {
        if (enabled) {
            this.io = io;;
        }
        this.enabled = enabled;
    }

    public void periodic() {
        if (!enabled)
            return;

        io.updateInputs(inputs);
        Logger.processInputs("CoralDetection", inputs);
    }

    public static CoralDetection getInstance() {
        if (instance == null)
            instance = new CoralDetection();
        return instance;
    }

    private CoralDetection() {
        camera = new PhotonCamera("Coral");
    }

    public void update() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty())
            return;

        PhotonPipelineResult result = results.get(results.size() - 1);

        hasTarget = result.hasTargets();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            yaw = Rotation2d.fromDegrees(target.getYaw() + 6);
        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public Rotation2d getYaw() {
        return yaw;
    }

    public Translation2d getFieldRelativeDir() {
        return new Translation2d(1, RobotState.getInstance().getGyroYaw().rotateBy(yaw.unaryMinus()));
    }

    public boolean isEnabled() {
        return enabled;
    }
}