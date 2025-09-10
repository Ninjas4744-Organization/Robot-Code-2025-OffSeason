package frc.robot.coraldetection;

import frc.lib.NinjasLib.controllers.Controller;

public class CoralDetectionIOCamera implements CoralDetectionIO {

    @Override
    public void updateInputs(CoralDetectionIOInputsAutoLogged inputs) {
        inputs.hasTarget = CoralDetection.getInstance().hasTarget();
        inputs.yaw = CoralDetection.getInstance().getYaw();
    }
}
