package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private double odometryDrift = 0;
    private boolean crashed;

    public VisionSubsystem() {
        Vision.setInstance(new Vision(Constants.Vision.kVisionConstants));
    }

    @Override
    public void periodic() {
        Vision.getInstance().periodic();

        Logger.recordOutput("Vision/Odometry Drift", odometryDrift);
        Logger.recordOutput("Vision/Crashed", crashed);
        odometryDrift += Swerve.getInstance().getOdometryTwist().getNorm() * Constants.Vision.kOdometryDriftPerMeter;

//        if (robotAcceleration >= Constants.Vision.kCrashAcceleration){
//            if(!crashed){
//                odometryDrift += Constants.Vision.kOdometryDriftPerCrash;
//                crashed = true;
//            }
//        } else
//            crashed = false;

        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
        for (VisionOutput estimation : estimations) {
            if(!estimation.hasTargets)
                continue;

            Logger.recordOutput("Vision/" + estimation.cameraName + "/Last Robot Pose", estimation.robotPose);

            Matrix<N3, N1> std = Constants.Vision.getVisionSTD(estimation);

            ChassisSpeeds speed = Swerve.getInstance().getChassisSpeeds(false);
            boolean passedFilters = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) <= Constants.Vision.kMaxSpeedFilter
                && speed.omegaRadiansPerSecond <= Constants.Vision.kMaxAngularSpeedFilter
                && estimation.closestTargetDist <= Constants.Vision.kMaxDistanceFilter
                && estimation.ambiguity <= Constants.Vision.kMaxAmbiguityFilter;

            Logger.recordOutput("Vision/" + estimation.cameraName + "/Passed Filters", passedFilters);
            if (passedFilters) {
                RobotState.getInstance().updateRobotPose(estimation, std);

                double P = odometryDrift * odometryDrift;
                double R = std.get(0, 0) * std.get(0, 0);
                double K = P / (P + R);

                odometryDrift = odometryDrift * Math.sqrt(1 - K);
            }
        }
    }

    public double getOdometryDrift() {
        return odometryDrift;
    }
}
