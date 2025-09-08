package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.NinjasLib.swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class CoralDetection {
    private static CoralDetection instance;
    private PhotonCamera camera;
    private boolean hasTargets;
    private Rotation2d yaw;
    private List<PhotonTrackedTarget> targets;
    private int currentTargetId = 0;

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

        hasTargets = result.hasTargets();
        if (!hasTargets)
            return;

        targets = result.getTargets();

        if(currentTargetId >= targets.size())
            currentTargetId = 0;

        for(int i = 0; i < targets.size(); i++) {
            if(targets.get(i).getArea() - targets.get(currentTargetId).getArea() > 2)
                currentTargetId = i;
        }

        for (int i = 0; i < targets.size(); i++) {
            Logger.recordOutput("Target" + i + " Area", targets.get(i).getArea());
        }

        Logger.recordOutput("Current Target Id", currentTargetId);
        Logger.recordOutput("Current Target Area", targets.get(currentTargetId));

        yaw = Rotation2d.fromDegrees(targets.get(currentTargetId).getYaw() + 3);
    }

    public boolean hasTargets() {
        return hasTargets;
    }

    public Rotation2d getYaw() {
        return yaw;
    }

    public Translation2d getFieldRelativeDir() {
        return new Translation2d(1, Swerve.getInstance().getGyro().getYaw().rotateBy(yaw.unaryMinus()));
    }
}
