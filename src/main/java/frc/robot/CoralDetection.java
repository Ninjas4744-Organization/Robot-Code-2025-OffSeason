package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class CoralDetection {
    private PhotonCamera camera;
    private boolean hasTarget;
    private Rotation2d yaw;

    public CoralDetection() {
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
            yaw = Rotation2d.fromDegrees(target.getYaw());
        }
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public Rotation2d getYaw() {
        return yaw;
    }

    public Translation2d getFieldRelativeDir() {
        return new Translation2d(1, yaw.unaryMinus().rotateBy(RobotState.getInstance().getRobotPose().getRotation()).rotateBy(Rotation2d.k180deg));
    }
}
