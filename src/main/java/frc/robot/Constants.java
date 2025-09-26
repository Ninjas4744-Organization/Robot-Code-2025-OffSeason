package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.SimpleControllerConstants;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.List;
import java.util.Map;

public class Constants {
    public enum RobotMode {
        /**
         * Running on a real robot
         */
        REAL,

        /** Running on a simulator */
        SIM,

        /** Replaying from a log file */
        REPLAY
    }

    public static class General {
        public static final RobotMode kSimMode = RobotMode.SIM;
        public static final RobotMode kRobotMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    //region Subsystems
    public static class Arm {
        public static final int kCanCoderID = 0;
        public static final double kCanCoderOffset = 0;
        public static final SensorDirectionValue kCanCoderReversed = SensorDirectionValue.Clockwise_Positive;

        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 40;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 60;
            kControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kControllerConstants.real.followers = new SimpleControllerConstants[1];
            kControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kControllerConstants.real.followers[0].id = 41;
            kControllerConstants.real.followers[0].inverted = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
            kControllerConstants.real.gearRatio = 50;
            kControllerConstants.real.conversionFactor = 2 * Math.PI;
            kControllerConstants.real.homePosition = Units.degreesToRadians(-90);
            kControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(3);

            /* Soft Limits */
            kControllerConstants.real.maxSoftLimit = Units.degreesToRadians(360);
            kControllerConstants.real.minSoftLimit = Units.degreesToRadians(-360);

            /* Hard Limit */
            kControllerConstants.real.isLimitSwitch = true;
            kControllerConstants.real.limitSwitchID = 1;
            kControllerConstants.real.limitSwitchDirection = -1;
            kControllerConstants.real.limitSwitchAutoStopReset = true;
            kControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }

        public enum Positions {
            Close(-90),
            L2(0),
            L3(0),
            L4(0),
            LowAlgaeOut(0),
            HighAlgaeOut(0),
            Net(70),
            Processor(0);

            final double angle;

            Positions(double angle) {
                this.angle = angle;
            }

            public double get() {
                return angle;
            }
        }
    }

    public static class Elevator {
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 30;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 60;
            kControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kControllerConstants.real.followers = new SimpleControllerConstants[1];
            kControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kControllerConstants.real.followers[0].id = 31;
            kControllerConstants.real.followers[0].inverted = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createPID(10, 0, 0, 0);
            kControllerConstants.real.gearRatio = 5;
            kControllerConstants.real.conversionFactor = Math.PI * 0.05;
            kControllerConstants.real.homePosition = 0;
            kControllerConstants.real.positionGoalTolerance = 0.01;

            /* Soft Limits */
            kControllerConstants.real.maxSoftLimit = 1.6;

            /* Hard Limit */
            kControllerConstants.real.isLimitSwitch = true;
            kControllerConstants.real.isVirtualLimit = true;
            kControllerConstants.real.virtualLimitStallThreshold = 30 / 12.0;
            kControllerConstants.real.limitSwitchID = 2;
            kControllerConstants.real.limitSwitchDirection = -1;
            kControllerConstants.real.limitSwitchAutoStopReset = true;
            kControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }

        public enum Positions {
            Close(0),
            L2(0.2),
            L3(0.6),
            L4(1),
            AlgaeReef(0.8),
            Net(1.2);

            final double height;

            Positions(double height) {
                this.height = height;
            }

            public double get() {
                return height;
            }
        }
    }

     public static class Outtake {
        public static final double kCurrentThreshold = 65;

        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 50;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 60;
            kControllerConstants.real.isBrakeMode = false;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Speeds {
            Intake(-1),
            Outtake(1),
            OuttakeAlgae(1);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class Intake {
        public static final int kBeamBreakerPort = 4;
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 20;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 80;
            kControllerConstants.real.isBrakeMode = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Speeds {
            Intake(-0.3),
            Outtake(0.6);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

     public static class IntakeAngle {
        public static final int kCanCoderID = 23;
        public static final double kCanCoderOffset = 1.331543;
        public static final SensorDirectionValue kCanCoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 21;
            kControllerConstants.real.main.inverted = true;
            kControllerConstants.real.currentLimit = 50;
            kControllerConstants.real.isBrakeMode = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 20, 25, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
            kControllerConstants.real.gearRatio = 65 + 1 / 3.0;
            kControllerConstants.real.conversionFactor = 2 * Math.PI;
            kControllerConstants.real.homePosition = Units.degreesToRadians(0);
            kControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(7);

            /* Soft Limits */
    //        kIntakeAngleControllerConstants.real.maxSoftLimit = Units.degreesToRadians(90);

            /* Hard Limit */
    //        kIntakeAngleControllerConstants.real.isLimitSwitch = true;
    //        kIntakeAngleControllerConstants.real.limitSwitchID = 3;
    //        kIntakeAngleControllerConstants.real.limitSwitchDirection = -1;
    //        kIntakeAngleControllerConstants.real.limitSwitchAutoStopReset = true;
    //        kIntakeAngleControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Positions {
            LOOK_DOWN(-23),
            LOOK_AT_L1(60),
            LOOK_AT_ARM(90);

            final double degrees;

            Positions(double degrees) {
                this.degrees = degrees;
            }

            public double get() {
                return degrees;
            }
        }
    }

    public static class IntakeAligner {
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 22;
            kControllerConstants.real.main.inverted = true;
            kControllerConstants.real.currentLimit = 80;
            kControllerConstants.real.isBrakeMode = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Speeds {
            Align(0.6);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class Climber {
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 60;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 80;
            kControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kControllerConstants.real.followers = new SimpleControllerConstants[1];
            kControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kControllerConstants.real.followers[0].id = 61;
            kControllerConstants.real.followers[0].inverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }
    }
    //endregion

    public static class Swerve {
        public static final double kDriverSpeedFactor = 1;
        public static final double kDriverRotationSpeedFactor = 1;
        public static final double kJoystickDeadband = 0.05;
        public static final boolean kDriverFieldRelative = true;

        public static final SwerveConstants kSwerveConstants = new SwerveConstants();
        static {
            /* Chassis */
            kSwerveConstants.chassis.trackWidth = 0.735;
            kSwerveConstants.chassis.wheelBase = 0.735;
            kSwerveConstants.chassis.bumperLength = 0.896;
            kSwerveConstants.chassis.bumperWidth = 0.896;
            kSwerveConstants.chassis.kinematics = new SwerveDriveKinematics(
                    new Translation2d(kSwerveConstants.chassis.wheelBase / 2.0, kSwerveConstants.chassis.trackWidth / 2.0),
                    new Translation2d(kSwerveConstants.chassis.wheelBase / 2.0, -kSwerveConstants.chassis.trackWidth / 2.0),
                    new Translation2d(-kSwerveConstants.chassis.wheelBase / 2.0, kSwerveConstants.chassis.trackWidth / 2.0),
                    new Translation2d(-kSwerveConstants.chassis.wheelBase / 2.0, -kSwerveConstants.chassis.trackWidth / 2.0)
            );

            /* Limits */
            kSwerveConstants.limits.maxSpeed = 4.5;
            kSwerveConstants.limits.maxAngularVelocity = 9.2;
            kSwerveConstants.limits.speedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationSpeedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.accelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationAccelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.maxSkidAcceleration = Double.MAX_VALUE;

            /* Modules */
            double wheelRadius = 0.048;
            kSwerveConstants.modules.openLoop = true;
            kSwerveConstants.modules.driveMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.driveMotorConstants.real.currentLimit = 100;
            kSwerveConstants.modules.driveMotorConstants.real.gearRatio = 5.9;
            kSwerveConstants.modules.driveMotorConstants.real.conversionFactor = wheelRadius * 2 * Math.PI;
            kSwerveConstants.modules.driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(90, 0.19);

            kSwerveConstants.modules.steerMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.steerMotorConstants.real.currentLimit = 60;
            kSwerveConstants.modules.steerMotorConstants.real.gearRatio = 18.75;
            kSwerveConstants.modules.steerMotorConstants.real.conversionFactor = 2 * Math.PI;
            kSwerveConstants.modules.steerMotorConstants.real.controlConstants = ControlConstants.createPID(10, 0, 0, 0);

            kSwerveConstants.modules.driveControllerType = Controller.ControllerType.TalonFX;
            kSwerveConstants.modules.steerControllerType = Controller.ControllerType.TalonFX;
            kSwerveConstants.modules.moduleConstants = new SwerveModuleConstants[4];

            for (int i = 0; i < 4; i++) {
                kSwerveConstants.modules.moduleConstants[i] = new SwerveModuleConstants(
                        i,
                        10 + i * 2,
                        11 + i * 2,
                        false,
                        false,
                        6 + i,
                        false,
                        0
                );
            }

            kSwerveConstants.modules.moduleConstants[0].CANCoderOffset = -0.218750;
            kSwerveConstants.modules.moduleConstants[1].CANCoderOffset = 0.232422;
            kSwerveConstants.modules.moduleConstants[2].CANCoderOffset = 0.229248;
            kSwerveConstants.modules.moduleConstants[3].CANCoderOffset = 0.210938;

            /* Gyro */
            kSwerveConstants.gyro.gyroID = 45;
            kSwerveConstants.gyro.gyroInverted = false;
            kSwerveConstants.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

            /* Simulation */
            kSwerveConstants.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
            kSwerveConstants.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);

            /* Special */
            kSwerveConstants.special.enableOdometryThread = true;
            kSwerveConstants.special.odometryThreadFrequency = 250;
            kSwerveConstants.special.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kSwerveConstants.special.robotStartPose = new Pose2d(3, 3, Rotation2d.kZero);
            kSwerveConstants.special.CANBus = "Swerve Bus";

            try {
                kSwerveConstants.special.robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
        static {
            kSwerveControllerConstants.swerveConstants = kSwerveConstants;
            kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(6, 0, 0.2, 0);
            kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(3, 0.5, 0.2, Units.degreesToRadians(15));
            kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
        }

        public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
            new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
            new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
        );
    }

    public static class Vision {
        public static final double kMaxDistanceFilter = 3;
        public static final double kMaxSpeedFilter = 3;
        public static final double kMaxAngularSpeedFilter = 7;
        public static final double kMaxAmbiguityFilter = 0.2;
        public static final double kOdometryDriftPerMeter = 0.02;
        public static final double kCrashAcceleration = 15;
        public static final double kOdometryDriftPerCrash = 0.5;

        public static final VisionConstants kVisionConstants = new VisionConstants();
        static {
            kVisionConstants.cameras = Map.of(
        //            "FrontRight", Pair.of(new Transform3d(0.0815 + 0.1054, -0.0745, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(-7.5 - 1.5))), VisionConstants.CameraType.PhotonVision),
        //            "FrontLeft", Pair.of(new Transform3d(0.0815 + 0.1054, 0.0755, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(7.5 - 1.5))), VisionConstants.CameraType.PhotonVision)
            "Right", Pair.of(new Transform3d(0.735 / 2, -0.03, 0, new Rotation3d(0, 0, 0)), VisionConstants.CameraType.PhotonVision)
        );

            kVisionConstants.fieldLayoutGetter = Constants.Field::getFieldLayoutWithIgnored;
            kVisionConstants.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kVisionConstants.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
        }

        public static Matrix<N3, N1> getVisionSTD(VisionOutput output) {
            double distStd = Math.pow(0.8 * output.closestTargetDist, 2) + 0.3;

            ChassisSpeeds speed = frc.lib.NinjasLib.swerve.Swerve.getInstance().getChassisSpeeds(false);
            double xySpeedStd = 6 * output.latency * Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
            double angleSpeedStd = 6 * output.latency * speed.omegaRadiansPerSecond;

            double xyStd = distStd + xySpeedStd;
            double angleStd = distStd + angleSpeedStd;

            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ latency", output.latency);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ dist", output.closestTargetDist);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ distStd", distStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ vel", Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond));
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ velStd", xySpeedStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ angleVel", speed.omegaRadiansPerSecond);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ angleVelStd", angleSpeedStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ xyStd", xyStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ angleStd", angleStd);
            return VecBuilder.fill(xyStd, xyStd, angleStd);
        }
    }

    public static class Field {
        public static AprilTagFieldLayout kBlueFieldLayout;
        public static AprilTagFieldLayout kRedFieldLayout;

        static {
            try {
                kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
                kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

                kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
                kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            } catch (IOException e) {
                throw new RuntimeException("Unable to load field layout");
            }
        }

        public static AprilTagFieldLayout getFieldLayoutWithIgnored(List<Integer> ignoredTags) {
            AprilTagFieldLayout layout;

            layout = RobotState.getAlliance() == DriverStation.Alliance.Blue
                    ? kBlueFieldLayout
                    : kRedFieldLayout;

            if (!ignoredTags.isEmpty()) {
                List<AprilTag> tags = layout.getTags();
                tags.removeIf(tag -> ignoredTags.contains(tag.ID));
                layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
            }

            return layout;
        }

        public static AprilTagFieldLayout getFieldLayoutWithAllowed(List<Integer> allowedTags) {
            AprilTagFieldLayout layout = getFieldLayout();
            if (!allowedTags.isEmpty()) {
                List<AprilTag> tags = layout.getTags();
                tags.removeIf(tag -> !allowedTags.contains(tag.ID));
                layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
            }

            return layout;
        }

        public static AprilTagFieldLayout getFieldLayout() {
            return getFieldLayoutWithIgnored(List.of());
        }

        public static Pose3d getTagPose(int id) {
            return getFieldLayout().getTagPose(id).get();
        }
    }

    public static class AutoDrive {
        public static final double kAutoDriveDistFromReef = 0.5;
        public static final double kAutoDriveRightSideOffset = 0.25;
        public static final double kAutoDriveLeftSideOffset = 0.25;
        public static final double kAutoDriveDistThreshold = 0.3;
    }
}
