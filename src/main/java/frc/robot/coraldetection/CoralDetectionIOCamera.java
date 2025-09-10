package frc.robot.coraldetection;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class CoralDetectionIOCamera implements CoralDetectionIO {
    private PhotonCamera camera;
    private List<PhotonTrackedTarget> targets;
    private int currentTargetId = 0;

    public CoralDetectionIOCamera() {
        camera = new PhotonCamera("Coral");
    }
    
    @Override
    public void updateInputs(CoralDetectionIOInputsAutoLogged inputs) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty())
            return;

        PhotonPipelineResult result = results.get(results.size() - 1);

        inputs.hasTargets = result.hasTargets();
        if (!inputs.hasTargets)
            return;

        targets = result.getTargets();

//        if(currentTargetId >= targets.size())
//            currentTargetId = 0;
//
//        for(int i = 0; i < targets.size(); i++) {
//            if(targets.get(i).getArea() - targets.get(currentTargetId).getArea() > 2)
//                currentTargetId = i;
//        }
//
//        for (int i = 0; i < targets.size(); i++) {
//            Logger.recordOutput("Target" + i + " Area", targets.get(i).getArea());
//        }
//
//        Logger.recordOutput("Current Target Id", currentTargetId);
//        Logger.recordOutput("Current Target Area", targets.get(currentTargetId));

//        yaw = Rotation2d.fromDegrees(targets.get(currentTargetId).getYaw() + 3);
        inputs.yaw = Rotation2d.fromDegrees(targets.get(0).getYaw() + 3);
    }
}
