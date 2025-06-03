package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.NinjasLib.dataclasses.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.RobotState;

public class FOMCalculator {
    private double odometryFOM = 1;
    private double[] visionFOM;
    private Pose2d lastRobotPose = new Pose2d();

    public void update(VisionOutput[] estimations) {
        if (visionFOM == null)
            visionFOM = new double[estimations.length];

        odometryFOM += RobotState.getInstance().getDistance(lastRobotPose) * 0.01;
        lastRobotPose = RobotState.getInstance().getRobotPose();

        ChassisSpeeds chassisSpeeds = Swerve.getInstance().getChassisSpeeds(false);
        double robotSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        for (int i = 0; i < estimations.length; i++) {
            visionFOM[i] = estimations[i].closestTagDist * 2 + robotSpeed * 3;

            if (visionFOM[i] < Constants.kResetOdometryFOMThreshold)
                odometryFOM = visionFOM[i];
        }
    }

    public double getOdometryFOM() {
        return odometryFOM;
    }

    public double getVisionFOM() {
        return visionFOM;
    }
}
