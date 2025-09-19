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
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import org.json.simple.parser.ParseException;

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
        public static final int kArmCanCoderID = 0;
        public static final double kArmCanCoderOffset = 0;
        public static final SensorDirectionValue kArmCanCoderReversed = SensorDirectionValue.Clockwise_Positive;

        public static final ControllerConstants kArmControllerConstants = new ControllerConstants();

    //endregion    static {
            /* Base */
            kArmControllerConstants.real.main.id = 40;
            kArmControllerConstants.real.main.inverted = false;
            kArmControllerConstants.real.currentLimit = 60;
            kArmControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kArmControllerConstants.real.followers = new SimpleControllerConstants[1];
            kArmControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kArmControllerConstants.real.followers[0].id = 41;
            kArmControllerConstants.real.followers[0].inverted = true;

            /* Control */
            kArmControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
            kArmControllerConstants.real.gearRatio = 50;
            kArmControllerConstants.real.conversionFactor = 2 * Math.PI;
            kArmControllerConstants.real.homePosition = Units.degreesToRadians(-90);
            kArmControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(3);

            /* Soft Limits */
            kArmControllerConstants.real.maxSoftLimit = Units.degreesToRadians(360);
            kArmControllerConstants.real.minSoftLimit = Units.degreesToRadians(-360);

            /* Hard Limit */
            kArmControllerConstants.real.isLimitSwitch = true;
            kArmControllerConstants.real.limitSwitchID = 1;
            kArmControllerConstants.real.limitSwitchDirection = -1;
            kArmControllerConstants.real.limitSwitchAutoStopReset = true;
            kArmControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kArmControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }
        public enum ArmPositions {
            Close(-90),
            L2(0),
            L3(0),
            L4(0),
            LowAlgaeOut(0),
            HighAlgaeOut(0),
            Net(70),
            Processor(0);

            final double angle;

            ArmPositions(double angle) {
                this.angle = angle;
            }

            public double get() {
                return angle;
            }
        }
    }

    public static class Elevator {
        public static final ControllerConstants kElevatorControllerConstants = new ControllerConstants();

        static {
            /* Base */
            kElevatorControllerConstants.real.main.id = 30;
            kElevatorControllerConstants.real.main.inverted = false;
            kElevatorControllerConstants.real.currentLimit = 60;
            kElevatorControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kElevatorControllerConstants.real.followers = new SimpleControllerConstants[1];
            kElevatorControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kElevatorControllerConstants.real.followers[0].id = 31;
            kElevatorControllerConstants.real.followers[0].inverted = true;

            /* Control */
            kElevatorControllerConstants.real.controlConstants = ControlConstants.createPID(10, 0, 0, 0);
            kElevatorControllerConstants.real.gearRatio = 5;
            kElevatorControllerConstants.real.conversionFactor = Math.PI * 0.05;
            kElevatorControllerConstants.real.homePosition = 0;
            kElevatorControllerConstants.real.positionGoalTolerance = 0.01;

            /* Soft Limits */
            kElevatorControllerConstants.real.maxSoftLimit = 1.6;

            /* Hard Limit */
            kElevatorControllerConstants.real.isLimitSwitch = true;
            kElevatorControllerConstants.real.isVirtualLimit = true;
            kElevatorControllerConstants.real.virtualLimitStallThreshold = 30 / 12.0;
            kElevatorControllerConstants.real.limitSwitchID = 2;
            kElevatorControllerConstants.real.limitSwitchDirection = -1;
            kElevatorControllerConstants.real.limitSwitchAutoStopReset = true;
            kElevatorControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kElevatorControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }
        public enum ElevatorPositions {
            Close(0),
            L2(0.2),
            L3(0.6),
            L4(1),
            AlgaeReef(0.8),
            Net(1.2);

            final double height;

            ElevatorPositions(double height) {
                this.height = height;
            }

            public double get() {
                return height;
            }
        }
    }

     public static class Outtake {
        public static final double kOuttakeCurrentThreshold = 65;
        public static final ControllerConstants kOuttakeControllerConstants = new ControllerConstants();

        static {
            /* Base */
            kOuttakeControllerConstants.real.main.id = 50;
            kOuttakeControllerConstants.real.main.inverted = false;
            kOuttakeControllerConstants.real.currentLimit = 60;
            kOuttakeControllerConstants.real.isBrakeMode = false;

            /* Simulation */
            kOuttakeControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }
        public enum OuttakeSpeeds {
            Intake(-1),
            Outtake(1),
            OuttakeAlgae(1);

            final double speed;

            OuttakeSpeeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class Intake {
        public static final int kIntakeBeamBreakerPort = 4;
        public static final ControllerConstants kIntakeControllerConstants = new ControllerConstants();

        static {
            /* Base */
            kIntakeControllerConstants.real.main.id = 20;
            kIntakeControllerConstants.real.main.inverted = false;
            kIntakeControllerConstants.real.currentLimit = 80;
            kIntakeControllerConstants.real.isBrakeMode = true;

            /* Simulation */
            kIntakeControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }
        public enum IntakeSpeeds {
            Intake(-0.3),
            Outtake(0.6);

            final double speed;

            IntakeSpeeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

     public static class IntakeAngle {
        public static final int kIntakeAngleCanCoderID = 23;
    public static final double kIntakeAngleCanCoderOffset = -0.441650;
    public static final SensorDirectionValue kIntakeAngleCanCoderReversed = SensorDirectionValue.CounterClockwise_Positive;
    public static final ControllerConstants kIntakeAngleControllerConstants = new ControllerConstants();

        static {
            /* Base */
            kIntakeAngleControllerConstants.real.main.id = 21;
            kIntakeAngleControllerConstants.real.main.inverted = true;
            kIntakeAngleControllerConstants.real.currentLimit = 50;
            kIntakeAngleControllerConstants.real.isBrakeMode = true;

            /* Control */
            kIntakeAngleControllerConstants.real.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 20, 25, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
            kIntakeAngleControllerConstants.real.gearRatio = 65 + 1 / 3.0;
            kIntakeAngleControllerConstants.real.conversionFactor = 2 * Math.PI;
            kIntakeAngleControllerConstants.real.homePosition = Units.degreesToRadians(0);
            kIntakeAngleControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(7);

            /* Soft Limits */
    //        kIntakeAngleControllerConstants.real.maxSoftLimit = Units.degreesToRadians(90);

            /* Hard Limit */
    //        kIntakeAngleControllerConstants.real.isLimitSwitch = true;
    //        kIntakeAngleControllerConstants.real.limitSwitchID = 3;
    //        kIntakeAngleControllerConstants.real.limitSwitchDirection = -1;
    //        kIntakeAngleControllerConstants.real.limitSwitchAutoStopReset = true;
    //        kIntakeAngleControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kIntakeAngleControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }
        public enum IntakeAnglePositions {
            LOOK_DOWN(0),
            LOOK_AT_L1(45),
            LOOK_AT_ARM(90);

            final double degrees;

            IntakeAnglePositions(double degrees) {
                this.degrees = degrees;
            }

            public double get() {
                return degrees;
            }
        }
    }

    public static class IntakeAligner {
        public static final ControllerConstants kIntakeAlignerControllerConstants = new ControllerConstants();

        static {
            /* Base */
            kIntakeAlignerControllerConstants.real.main.id = 22;
            kIntakeAlignerControllerConstants.real.main.inverted = true;
            kIntakeAlignerControllerConstants.real.currentLimit = 80;
            kIntakeAlignerControllerConstants.real.isBrakeMode = true;

            /* Simulation */
            kIntakeAlignerControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum IntakeAlignerSpeeds {
            Align(0.6);

            final double speed;

            IntakeAlignerSpeeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class Climber {
        public static final ControllerConstants kClimberControllerConstants = new ControllerConstants();

        static {
            /* Base */
            kClimberControllerConstants.real.main.id = 60;
            kClimberControllerConstants.real.main.inverted = false;
            kClimberControllerConstants.real.currentLimit = 80;
            kClimberControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kClimberControllerConstants.real.followers = new SimpleControllerConstants[1];
            kClimberControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kClimberControllerConstants.real.followers[0].id = 61;
            kClimberControllerConstants.real.followers[0].inverted = true;

            /* Simulation */
            kClimberControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }
    }
    //endregion
    //endregion


    //region Positions
    public enum ArmPositions {
        Close(-90),
        L2(0),
        L3(0),
        L4(0),
        LowAlgaeOut(0),
        HighAlgaeOut(0),
        Net(70),
        Processor(0);

        final double angle;

        ArmPositions(double angle) {
            this.angle = angle;
        }

        public double get() {
            return angle;
        }
    }

    public enum ElevatorPositions {
        Close(0),
        L2(0.2),
        L3(0.6),
        L4(1),
        AlgaeReef(0.8),
        Net(1.2);

        final double height;

        ElevatorPositions(double height) {
            this.height = height;
        }

        public double get() {
            return height;
        }
    }

    public enum IntakeAnglePositions {
        LOOK_DOWN(-16),
        LOOK_AT_L1(60),
        LOOK_AT_ARM(90);

        final double degrees;

        IntakeAnglePositions(double degrees) {
            this.degrees = degrees;
        }

        public double get() {
            return degrees;
        }
    }

    public enum OuttakeSpeeds {
        Intake(-1),
        Outtake(1),
        OuttakeAlgae(1);

        final double speed;

        OuttakeSpeeds(double speed) {
            this.speed = speed;
        }

        public double get() {
            return speed;
        }
    }

    public enum IntakeSpeeds {
        Intake(-0.3),
        Outtake(0.6);

        final double speed;

        IntakeSpeeds(double speed) {
            this.speed = speed;
        }

        public double get() {
            return speed;
        }
    }

    public enum IntakeAlignerSpeeds {
        Align(0.6);

        final double speed;

        IntakeAlignerSpeeds(double speed) {
            this.speed = speed;
        }

        public double get() {
            return speed;
        }
    }


    public static class Swerve {
        public static final double kDriverSpeedFactor = 1;
        public static final double kDriverRotationSpeedFactor = 1;
        public static final double kJoystickDeadband = 0.05;
        public static final boolean kDriverFieldRelative = true;

        public static final SwerveConstants kSwerveConstants = new SwerveConstants();
        static {
            // Move chassis-related settings to chassis subclass
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

            // Move limits-related settings to limits subclass
            kSwerveConstants.limits.maxSpeed = 4.5;
            kSwerveConstants.limits.maxAngularVelocity = 9.2;
            kSwerveConstants.limits.speedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationSpeedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.accelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationAccelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.maxSkidAcceleration = Double.MAX_VALUE;

            // Move modules-related settings to modules subclass
            kSwerveConstants.modules.openLoop = true;
            double wheelRadius = 0.048;
            kSwerveConstants.modules.driveMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.driveMotorConstants.real.currentLimit = 100;
            kSwerveConstants.modules.driveMotorConstants.real.gearRatio = 5.9;
            kSwerveConstants.modules.driveMotorConstants.real.conversionFactor = wheelRadius * 2 * Math.PI;
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
                kSwerveConstants.modules.moduleConstants[i].moduleNumber = i;
                kSwerveConstants.modules.moduleConstants[i].driveMotorID = 10 + i * 2;
                kSwerveConstants.modules.moduleConstants[i].driveMotorInverted = false;
                kSwerveConstants.modules.moduleConstants[i].steerMotorID = 11 + i * 2;
                kSwerveConstants.modules.moduleConstants[i].steerMotorInverted = false;
                kSwerveConstants.modules.moduleConstants[i].canCoderID = 6 + i;
                kSwerveConstants.modules.moduleConstants[i].invertCANCoder = false;
            }

            kSwerveConstants.modules.moduleConstants[0].CANCoderOffset = -0.218750;
            kSwerveConstants.modules.moduleConstants[1].CANCoderOffset = 0.232422;
            kSwerveConstants.modules.moduleConstants[2].CANCoderOffset = 0.229248;
            kSwerveConstants.modules.moduleConstants[3].CANCoderOffset = 0.210938;

            // Move gyro-related settings to gyro subclass
            kSwerveConstants.gyro.gyroID = 45;
            kSwerveConstants.gyro.gyroInverted = false;
            kSwerveConstants.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

            // Move simulation-related settings to simulation subclass
            kSwerveConstants.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
            kSwerveConstants.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);

            // Move special-related settings to special subclass
            kSwerveConstants.special.enableOdometryThread = true;
            kSwerveConstants.special.odometryThreadFrequency = 250;
            kSwerveConstants.special.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kSwerveConstants.special.robotStartPose = new Pose2d(3, 3, Rotation2d.kZero);

            // Move CAN-related settings
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

        public static final PathFollowingController kAutonomyConfig =
            new PPHolonomicDriveController(
                    new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
                    new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
            );
    }

    public static class Vision {
        public static final VisionConstants kVisionConstants = new VisionConstants();

        static {
            kVisionConstants.cameras = Map.of(
        //            "FrontRight", Pair.of(new Transform3d(0.0815 + 0.1054, -0.0745, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(-7.5 - 1.5))), VisionConstants.CameraType.PhotonVision),
        //            "FrontLeft", Pair.of(new Transform3d(0.0815 + 0.1054, 0.0755, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(7.5 - 1.5))), VisionConstants.CameraType.PhotonVision)
            "Right", Pair.of(new Transform3d(0.735 / 2, -0.03, 0, new Rotation3d(0, 0, 0)), VisionConstants.CameraType.PhotonVision)
        );

            kVisionConstants.maxAmbiguity = 0.2;
            kVisionConstants.maxDistance = 5;
            kVisionConstants.fieldLayoutGetter = Constants.Field::getFieldLayoutWithIgnored;
            kVisionConstants.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kVisionConstants.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
        }

        public static Matrix<N3, N1> getVisionSTD(VisionOutput output) {
            double distStd = Math.pow(0.4 * output.closestTargetDist, 2) + 0.3;

        ChassisSpeeds speed = Swerve.getInstance().getChassisSpeeds(false);
        double latMs = 50; //40ms lat + 10ms from 40fps

        double xyStd = distStd + 2 * (latMs / 1000) * Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
        double angleStd = distStd + 2 * (latMs / 1000) * speed.omegaRadiansPerSecond;

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

//    //region Standard Deviations
//    public static final double kOdometrySTDPerMeter = 0.02;
//    public static final double kCrashedAccelerationThreshold = 15;
//    public static final double kCrashedOdometrySTDBonus = 4;
//    public static final double kResetOdometrySTDThreshold = 2.5;
//    public static final double kOdometrySTDResetValue = 0.1;
//    public static final int kCyclesToOdometrySTDReset = 35;
//
//    public static final double kVisionSTDGood = 0.9;
//    public static final double kVisionSTDDistMultiplier = 1;
//    public static final double kVisionSTDGoodDist = 1.5;
//    public static final double kVisionSTDSpeedMultiplier = 1;
//    public static final double kVisionSTDGoodSpeed = 2;
//    public static final double kVisionSTDAngularSpeedMultiplier = 1;
//    public static final double kVisionSTDGoodAngularSpeed = 2;

    public static class AutoDrive {
        public static final double kAutoDriveDistFromReef = 0.5;
        public static final double kAutoDriveRightSideOffset = 0.25;
        public static final double kAutoDriveLeftSideOffset = 0.25;
        public static final double kAutoDriveDistThreshold = 0.3;
    }
}
