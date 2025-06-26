package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.dataclasses.*;
import frc.lib.NinjasLib.dataclasses.RealControllerConstants.SimpleControllerConstants;
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

    /* General */
    public static final RobotMode kSimMode = RobotMode.SIM;
    public static final RobotMode kCurrentMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    /* Subsystems */
    public static final ControllerConstants kArmControllerConstants = new ControllerConstants();
    static {
        /* Base */
        kArmControllerConstants.real.main.id = 20;
        kArmControllerConstants.real.main.inverted = false;
        kArmControllerConstants.real.currentLimit = 80;
        kArmControllerConstants.real.isBrakeMode = true;

        /* Followers */
        kArmControllerConstants.real.followers = new SimpleControllerConstants[1];
        kArmControllerConstants.real.followers[0] = new SimpleControllerConstants();
        kArmControllerConstants.real.followers[0].id = 21;
        kArmControllerConstants.real.followers[0].inverted = true;

        /* Control */
        kArmControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
        kArmControllerConstants.real.gearRatio = 50;
        kArmControllerConstants.real.conversionFactor = 2 * Math.PI;
        kArmControllerConstants.real.homePosition = Units.degreesToRadians(-60);
        kArmControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(1.5);

        /* Soft Limits */
        kArmControllerConstants.real.maxSoftLimit = Units.degreesToRadians(240);

        /* Hard Limit */
        kArmControllerConstants.real.isLimitSwitch = true;
        kArmControllerConstants.real.limitSwitchID = 2;
        kArmControllerConstants.real.limitSwitchDirection = -1;
        kArmControllerConstants.real.limitSwitchAutoStopReset = true;
        kArmControllerConstants.real.limitSwitchInverted = true;

        /* Simulation */
        kArmControllerConstants.motorType = DCMotor.getKrakenX60(2);
    }

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
        kElevatorControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
        kElevatorControllerConstants.real.gearRatio = 5;
        kElevatorControllerConstants.real.conversionFactor = Math.PI * 0.05;
        kElevatorControllerConstants.real.homePosition = 0;
        kElevatorControllerConstants.real.positionGoalTolerance = 0.01;

        /* Soft Limits */
        kElevatorControllerConstants.real.maxSoftLimit = 1.6;

        /* Hard Limit */
        kElevatorControllerConstants.real.isLimitSwitch = true;
        kElevatorControllerConstants.real.limitSwitchID = 3;
        kElevatorControllerConstants.real.limitSwitchDirection = -1;
        kElevatorControllerConstants.real.limitSwitchAutoStopReset = true;
        kElevatorControllerConstants.real.limitSwitchInverted = true;

        /* Simulation */
        kElevatorControllerConstants.motorType = DCMotor.getKrakenX60(2);
    }

    /* Positions */
    public enum ArmPositions {
        Close(0),
        Open(180),
        Mid(90);

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
        Open(1.5),
        Mid(0.75);

        final double height;

        ElevatorPositions(double height) {
            this.height = height;
        }

        public double get() {
            return height;
        }
    }

    /* Swerve */
    public static final double kDriverSpeedFactor = 1;
    public static final double kDriverRotationSpeedFactor = 1;

    public static final double kJoystickDeadband = 0.05;
    public static final boolean kInvertGyro = false;
    public static final boolean kDriverFieldRelative = true;

    public static final SwerveConstants kSwerveConstants = new SwerveConstants();

    static {
        kSwerveConstants.openLoop = true;
        kSwerveConstants.trackWidth = 0.685;
        kSwerveConstants.wheelBase = 0.685;
        kSwerveConstants.bumperLength = 0.846;
        kSwerveConstants.bumperWidth = 0.846;
        kSwerveConstants.kinematics = new SwerveDriveKinematics(
            new Translation2d(kSwerveConstants.wheelBase / 2.0, kSwerveConstants.trackWidth / 2.0),
            new Translation2d(kSwerveConstants.wheelBase / 2.0, -kSwerveConstants.trackWidth / 2.0),
            new Translation2d(-kSwerveConstants.wheelBase / 2.0, kSwerveConstants.trackWidth / 2.0),
            new Translation2d(-kSwerveConstants.wheelBase / 2.0, -kSwerveConstants.trackWidth / 2.0)
        );

        kSwerveConstants.maxSpeed = 4.5;
        kSwerveConstants.maxAngularVelocity = 9.2;
//        kSwerveConstants.maxAcceleration = Double.MAX_VALUE;
        kSwerveConstants.maxSkidAcceleration = 100;
        kSwerveConstants.speedLimit = 4.5;
        kSwerveConstants.rotationSpeedLimit = 9.2;
        kSwerveConstants.accelerationLimit = Double.MAX_VALUE;//11.8;
        kSwerveConstants.rotationAccelerationLimit = Double.MAX_VALUE;//63.79;

        kSwerveConstants.moduleConstants = new SwerveModuleConstants[4];

        double wheelRadius = 0.049806;//0.048;

        for (int i = 0; i < 4; i++) {
            kSwerveConstants.moduleConstants[i] = new SwerveModuleConstants(i,
                    new ControllerConstants(),
                    new ControllerConstants(),
                kSwerveConstants.maxSpeed,
                6 + i,
                Controller.ControllerType.TalonFX,
                Controller.ControllerType.TalonFX,
                false,
                0);

            kSwerveConstants.moduleConstants[i].driveMotorConstants.real.main.id = 10 + i * 2;
//            kSwerveConstants.moduleConstants[i].driveMotorConstants.main.inverted = i % 2 == 0;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.real.currentLimit = 72;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.real.gearRatio = 5.360;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.real.conversionFactor = wheelRadius * 2 * Math.PI;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(5 / 0.056267331109070916, 0.19);

            kSwerveConstants.moduleConstants[i].angleMotorConstants.real.main.id = 11 + i * 2;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.real.currentLimit = 60;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.real.gearRatio = 18.75;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.real.conversionFactor = 2 * Math.PI;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.real.controlConstants = ControlConstants.createPID(5, 0, 0, 0);
        }

        kSwerveConstants.moduleConstants[0].CANCoderOffset = -0.295654;
        kSwerveConstants.moduleConstants[1].CANCoderOffset = 0.230713;
        kSwerveConstants.moduleConstants[2].CANCoderOffset = 0.238037;
        kSwerveConstants.moduleConstants[3].CANCoderOffset = 0.273438;

        try {
            kSwerveConstants.robotConfig = RobotConfig.fromGUISettings();
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }

        kSwerveConstants.driveMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerveConstants.steerMotorType = DCMotor.getKrakenX60Foc(1);
    }

    public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
    static {
        kSwerveControllerConstants.swerveConstants = kSwerveConstants;
        kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(6, 0, 0.2, 0);
        kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(3, 0.5, 0.2, Units.degreesToRadians(15));
        kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
        kSwerveControllerConstants.enableRotationPIDCorrection = true;
    }

    public static final PathFollowingController kAutonomyConfig =
        new PPHolonomicDriveController(
            new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
            new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
        );

    /* Vision */
    public static final double kResetOdometryFOMThreshold = 2.5;
    public static final VisionConstants kVisionConstants = new VisionConstants();

    static {
        kVisionConstants.cameras = Map.of(
                "FrontRight", Pair.of(new Transform3d(0.0815 + 0.1054, -0.0745, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(-7.5 - 1.5))), VisionConstants.CameraType.PhotonVision),
                "FrontLeft", Pair.of(new Transform3d(0.0815 + 0.1054, 0.0755, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(7.5 - 1.5))), VisionConstants.CameraType.PhotonVision)
        );

        kVisionConstants.maxAmbiguity = 0.2;
        kVisionConstants.maxDistance = 2;
        kVisionConstants.fieldLayoutGetter = Constants::getFieldLayoutWithIgnored;
    }

    /* Field */
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
