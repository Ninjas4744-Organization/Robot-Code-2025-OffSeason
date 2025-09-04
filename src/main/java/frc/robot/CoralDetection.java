package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.NinjasLib.swerve.Swerve;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class CoralDetection {
    private PhotonCamera camera;
    private boolean hasTarget;
    private Rotation2d yaw;
    private static CoralDetection instance;

    public static CoralDetection getInstance() {
        if (instance == null)
            instance = new CoralDetection();
        return instance;
    }

    private CoralDetection() {
        if(Robot.isReal())
            camera = new PhotonCamera("Coral");
    }

    public void update() {
        if(Robot.isSimulation())
            return;

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
        return new Translation2d(1, Swerve.getInstance().getGyro().getYaw().rotateBy(yaw.unaryMinus()));
    }
}
