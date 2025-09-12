package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class STDDevCalculator {
    private double odometrySTDDev = 1;
    private List<Double> visionSTDDev;
    private boolean crashed = false;
    private int amountOfEstimations = 0;
    private int cyclesUntilReset = Constants.kCyclesToOdometrySTDReset;

    private double stdFunc(double x, double x0, double a) {
        return Math.max(1, a * Math.pow(x / x0, 2) - a + 1);
    }

    public void update(VisionOutput[] estimations) {
        if(cyclesUntilReset >= 0)
            cyclesUntilReset--;

        if (cyclesUntilReset == 0)
            odometrySTDDev = Constants.kOdometrySTDResetValue;

        visionSTDDev = new ArrayList<>();
        amountOfEstimations = estimations.length;

        odometrySTDDev += Swerve.getInstance().getOdometryTwist().getNorm() * Constants.kOdometrySTDPerMeter;

        double acc = Math.hypot(Swerve.getInstance().getGyro().getAccelerationX(), Swerve.getInstance().getGyro().getAccelerationY());
        if (acc > Constants.kCrashedAccelerationThreshold) {
            if (!crashed) {
                odometrySTDDev += Constants.kCrashedOdometrySTDBonus;
                crashed = true;
            }
        } else
            crashed = false;

        ChassisSpeeds chassisSpeeds = Swerve.getInstance().getChassisSpeeds(false);
        double robotSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        double robotAngularSpeed = chassisSpeeds.omegaRadiansPerSecond;
        for (VisionOutput estimation : estimations) {
            if (!estimation.hasTargets)
                continue;

            visionSTDDev.add(Constants.kVisionSTDGood * stdFunc(estimation.closestTargetDist, Constants.kVisionSTDGoodDist, Constants.kVisionSTDDistMultiplier)
                           * stdFunc(robotSpeed, Constants.kVisionSTDGoodSpeed, Constants.kVisionSTDSpeedMultiplier)
                           * stdFunc(robotAngularSpeed, Constants.kVisionSTDGoodAngularSpeed, Constants.kVisionSTDAngularSpeedMultiplier));
        }

        if (cyclesUntilReset < 0) {
            for (int i = 0; i < estimations.length; i++) {
                if (!estimations[i].hasTargets)
                    continue;

                if (visionSTDDev.get(i) < Constants.kResetOdometrySTDThreshold) {
                    cyclesUntilReset = Constants.kCyclesToOdometrySTDReset;
                    break;
                }
            }
        }

        Logger.recordOutput("STDs/Robot Speed", robotSpeed);
        Logger.recordOutput("STDs/Robot Angular Speed", robotAngularSpeed);
        Logger.recordOutput("STDs/Acceleration", acc);
        Logger.recordOutput("STDs/Crashed", crashed);
        Logger.recordOutput("STDs/Odometry STD", odometrySTDDev);

        int camEstCount = 1;
        String lastCamName = "-1";
        for (int i = 0; i < estimations.length; i++) {
            if (!estimations[i].hasTargets)
                continue;

            if (lastCamName.equals(estimations[i].cameraName))
                camEstCount++;
            else
                camEstCount = 1;

            String name = "STDs/" + estimations[i].cameraName + " " + camEstCount;
            Logger.recordOutput(name + "/STD", visionSTDDev.get(i));
            lastCamName = estimations[i].cameraName;
        }
    }

    public double getOdometrySTDDev() {
        if (DriverStation.isDisabled())
            return 1;
        return odometrySTDDev;
    }

    public double[] getVisionSTDDev() {
        if (DriverStation.isDisabled()) {
            double[] STDs = new double[amountOfEstimations];
            for (int i = 0; i < amountOfEstimations; i++)
                STDs[i] = 0.1;

            return STDs;
        }

        return visionSTDDev.stream().mapToDouble(Double::doubleValue).toArray();
    }
}
