package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class FOMCalculator {
    private double odometryFOM = 1;
    private List<Double> visionFOM;
    private boolean crashed = false;
    private int amountOfEstimations = 0;

    public void update(VisionOutput[] estimations) {
        visionFOM = new ArrayList<>();
        amountOfEstimations = estimations.length;

        odometryFOM += Swerve.getInstance().getOdometryTwist().getNorm() * Constants.kOdometryFOMPerMeter;

//        if (RobotState.getInstance().getAcceleration().getNorm() > Constants.kCrashedAccelerationThreshold) {
//            if (!crashed) {
//                odometryFOM += Constants.kCrashedOdometryFOMBonus;
//                crashed = true;
//            }
//        } else
//            crashed = false;

        ChassisSpeeds chassisSpeeds = Swerve.getInstance().getChassisSpeeds(false);
        double robotSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        double robotAngularSpeed = chassisSpeeds.omegaRadiansPerSecond;
        for (int i = 0; i < estimations.length; i++) {
            if (!estimations[i].hasTargets)
                continue;

            visionFOM.add(estimations[i].closestTargetDist * Constants.kVisionFOMDistMultiplier
                + robotSpeed * Constants.kVisionFOMSpeedMultiplier
                + robotAngularSpeed * Constants.kVisionFOMAngularSpeedMultiplier
                + Math.abs(Math.cos(estimations[i].cameraToClosestTargetTransform.getRotation().getZ())) * Constants.kVisionFOMAngleTransformMultiplier
                + Math.abs(Math.cos(estimations[i].cameraToClosestTargetTransform.getRotation().getY())) * Constants.kVisionFOMAngleTransformMultiplier
            );

            if (visionFOM.get(i) < Constants.kResetOdometryFOMThreshold) {
                if (Constants.kOdometryFOMResetValue < odometryFOM)
                    odometryFOM = Constants.kOdometryFOMResetValue;
            }
        }

        Logger.recordOutput("FOMs/Robot Speed", robotSpeed);
        Logger.recordOutput("FOMs/Robot Angular Speed", robotAngularSpeed);
//        Logger.recordOutput("FOMs/Acceleration", RobotState.getInstance().getAcceleration().getNorm());
        Logger.recordOutput("FOMs/Crashed", crashed);
        Logger.recordOutput("FOMs/Odometry FOM", odometryFOM);

        int camEstCount = 1;
        String lastCamName = "-1";
        for (int i = 0; i < estimations.length; i++) {
            if (lastCamName == estimations[i].cameraName)
                camEstCount++;
            else
                camEstCount = 1;

            String name = "FOMs/" + estimations[i].cameraName + " " + camEstCount;
            Logger.recordOutput(name + "/FOM", visionFOM.get(i));
            Logger.recordOutput(name + "/Yaw Transform", Math.abs(Math.cos(estimations[i].cameraToClosestTargetTransform.getRotation().getZ())));
            Logger.recordOutput(name + "/Pitch Transform", Math.abs(Math.cos(estimations[i].cameraToClosestTargetTransform.getRotation().getY())));
        }
    }

    public double getOdometryFOM() {
        if (DriverStation.isDisabled())
            return 1;
        return odometryFOM;
    }

    public double[] getVisionFOM() {
        if (DriverStation.isDisabled()) {
            double[] FOMs = new double[amountOfEstimations];
            for (int i = 0; i < amountOfEstimations; i++)
                FOMs[i] = 0.1;

            return FOMs;
        }

        return visionFOM.stream().mapToDouble(Double::doubleValue).toArray();
    }
}
