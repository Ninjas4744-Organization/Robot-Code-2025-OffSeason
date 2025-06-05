package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.dataclasses.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

public class FOMCalculator {
    private double odometryFOM = 1;
    private double[] visionFOM;
    private Pose2d lastRobotPose = new Pose2d();
    private boolean crashed = false;

    public void update(VisionOutput[] estimations) {
        if (visionFOM == null)
            visionFOM = new double[estimations.length];

        odometryFOM += Swerve.getInstance().getOdometryTwist().getNorm() * 0.05;
        lastRobotPose = RobotState.getInstance().getRobotPose();

        Logger.recordOutput("FOMs/Acc", RobotState.getInstance().getAcceleration().getNorm());
        if (RobotState.getInstance().getAcceleration().getNorm() > 15) {
            if (!crashed) {
                odometryFOM += 4;
                crashed = true;
            }
        } else
            crashed = false;

        ChassisSpeeds chassisSpeeds = Swerve.getInstance().getChassisSpeeds(false);
        double robotSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        for (int i = 0; i < estimations.length; i++) {
            if (!estimations[i].hasTargets)
                continue;

            visionFOM[i] = estimations[i].closestTagDist * 3.5 + robotSpeed;

            if (visionFOM[i] < Constants.kResetOdometryFOMThreshold) {
                if (visionFOM[i] / 2 < odometryFOM)
                    odometryFOM = visionFOM[i] / 2;
            }
        }

        Logger.recordOutput("FOMs/Odometry FOM", odometryFOM);
        Logger.recordOutput("FOMs/Vision FOM", visionFOM);
    }

    public double getOdometryFOM() {
        if (DriverStation.isDisabled())
            return 1;
        return odometryFOM;
    }

    public double[] getVisionFOM() {
        if (DriverStation.isDisabled())
            return new double[]{0.1, 0.1};
        return visionFOM;
    }
}
