package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.*;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveSubsystem extends SubsystemBase {
    private boolean enabled;
    private List<Pose2d> reefAprilTags;
    private Pose2d finalTarget;
    private Pose2d target;
    private Command driveToReefCommand;
    private Command driveToCoralCommand;

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(Constants.Swerve.kSwerveConstants));
            SwerveController.setInstance(new SwerveController(Constants.Swerve.kSwerveControllerConstants));
            SwerveController.getInstance().setChannel("Driver");

            AprilTagFieldLayout layout = Constants.Field.getFieldLayout();
            reefAprilTags = List.of(
                    layout.getTagPose(6).get().toPose2d(),
                    layout.getTagPose(7).get().toPose2d(),
                    layout.getTagPose(8).get().toPose2d(),
                    layout.getTagPose(9).get().toPose2d(),
                    layout.getTagPose(10).get().toPose2d(),
                    layout.getTagPose(11).get().toPose2d(),
                    layout.getTagPose(17).get().toPose2d(),
                    layout.getTagPose(18).get().toPose2d(),
                    layout.getTagPose(19).get().toPose2d(),
                    layout.getTagPose(20).get().toPose2d(),
                    layout.getTagPose(21).get().toPose2d(),
                    layout.getTagPose(22).get().toPose2d()
            );
            finalTarget = Pose2d.kZero;
        }
    }

    public void swerveDrive(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), Constants.Swerve.kJoystickDeadband) * Constants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), Constants.Swerve.kJoystickDeadband) * Constants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(rightX.getAsDouble(), Constants.Swerve.kJoystickDeadband) * Constants.Swerve.kDriverRotationSpeedFactor,
                        Constants.Swerve.kDriverFieldRelative
                )), "Driver");
    }

    private ProfiledPIDController autoReefAnglePID = new ProfiledPIDController(6, 0, 0.1, new TrapezoidProfile.Constraints(6, 12));
    double f(double error){
        double a = 2;
        double b = 2;
//        if(surpassedFirstThreshold()) {
//            a = 1.5;
//            b = 1.5;
//        }
        return Math.pow(a * error, 1 / b);
    }

    private boolean isAutoDriveReefStage1 = true;
    public Supplier<Command> autoDriveToReef(BooleanSupplier isRightSide) {
        return () -> {
            driveToReefCommand = Commands.sequence(
                    Commands.runOnce(() -> {
                        SwerveController.getInstance().setChannel("AutoReef");
                        finalTarget = RobotState.getInstance().getRobotPose().nearest(reefAprilTags);
                        finalTarget = new Pose2d(finalTarget.getTranslation(), finalTarget.getRotation().rotateBy(Rotation2d.k180deg));
                        finalTarget = finalTarget.transformBy(new Transform2d(RobotState.getL() == 4 ? -Constants.AutoDrive.kDistFromReefL4 : -Constants.AutoDrive.kDistFromReef, isRightSide.getAsBoolean() ? -Constants.AutoDrive.kRightSideOffset : Constants.AutoDrive.kLeftSideOffset, Rotation2d.kZero));
                        target = finalTarget.transformBy(new Transform2d(-Constants.AutoDrive.kDistBackFirstTarget, 0, Rotation2d.kZero));
                        autoReefAnglePID.reset(RobotState.getInstance().getRobotPose().getRotation().getRadians());
//                pidRotation.reset(RobotState.getInstance().getRobotPose().getRotation().getRadians());
                    }),
                    Commands.run(() -> {
                        if (atFirstTarget(target) && isAutoDriveReefStage1) {
                            target = finalTarget;
                            isAutoDriveReefStage1 = false;
                        }

                        Translation2d translation = RobotState.getInstance().getTranslation(target);
                        Translation2d dir = translation.div(translation.getNorm());
                        double velocity = f(translation.getNorm());
                        double anglePID = autoReefAnglePID.calculate(RobotState.getInstance().getRobotPose().getRotation().getRadians(), target.getRotation().getRadians());//SwerveController.getInstance().lookAt(target.getRotation().getRadians());
//                        Pose2d robotPose = RobotState.getInstance().getRobotPose();
//                Logger.recordOutput("dir", new Pose2d(robotPose.getX() + dir.getX() / 2, robotPose.getY() + dir.getY() / 2, new Rotation2d(dir.getX(), dir.getY())));
//                Logger.recordOutput("vel", velocity);
//                Logger.recordOutput("anglePID", anglePID);

                        SwerveController.getInstance().setControl(
                                new SwerveInput(
                                        velocity * dir.getX(),
                                        velocity * dir.getY(),
                                        anglePID,
                                        true
                                ), "AutoReef"
                        );
                    })/*.until(this::atGoal)*/.andThen(Commands.runOnce(() -> SwerveController.getInstance().setControl(new SwerveInput(0, 0, 0, false), "AutoReef")))
            );
            return driveToReefCommand;
        };
    }

    public double distFromGoal() {
        return Math.abs(RobotState.getInstance().getDistance(finalTarget));
    }

    private boolean atFirstTarget(Pose2d target) {
        return Math.abs(RobotState.getInstance().getDistance(target)) < Constants.AutoDrive.kFirstDistThreshold;
    }

    public boolean atGoal() {
        return Math.abs(RobotState.getInstance().getDistance(finalTarget)) < Constants.AutoDrive.kDistThreshold
                && Math.abs(finalTarget.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getRadians()) < Constants.AutoDrive.kAngleThreshold.getRadians();
    }

//    private boolean surpassedFirstThreshold() {
//        return Math.abs(RobotState.getInstance().getDistance(finalTarget)) < Constants.AutoDrive.kAutoDriveDistFirstThreshold;
//    }

    private Timer noCoralTimer = new Timer();
    public Command driveToCoral() {
        driveToCoralCommand = Commands.sequence(
                Commands.waitUntil(() -> RobotContainer.getCoralDetection().hasTargets()),
                Commands.runOnce(() -> {
                    SwerveController.getInstance().setChannel("AutoCoral");
                    StateMachine.getInstance().changeRobotState(States.INTAKE_CORAL);
                }),
                Commands.run(() -> {
                    Translation2d dir = RobotContainer.getCoralDetection().getFieldRelativeDir();

                    double anglePID = SwerveController.getInstance().lookAt(dir);
                    double driveSpeed = 1;

                    SwerveController.getInstance().setControl(
                            new SwerveInput(
                                    dir.getX() * driveSpeed,
                                    dir.getY() * driveSpeed,
                                    anglePID,
                                    true),
                            "AutoCoral"
                    );
                }).until(() -> {
                    if (RobotState.getInstance().getRobotState() == States.CORAL_IN_INTAKE)
                        return true;

                    if (!RobotContainer.getCoralDetection().hasTargets() && !noCoralTimer.isRunning())
                        noCoralTimer.restart();

                    if (RobotContainer.getCoralDetection().hasTargets()) {
                        noCoralTimer.stop();
                        noCoralTimer.reset();
                    }

                    if (noCoralTimer.get() > 1)
                        return true;

                    return false;
                }),
                close()
        );

        return driveToCoralCommand;
    }

    public void stopAutoDriving() {
        if (driveToReefCommand != null)
            driveToReefCommand.cancel();

        if (driveToCoralCommand != null)
            driveToCoralCommand.cancel();
    }

    public Command close() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            isAutoDriveReefStage1 = true;
            stopAutoDriving();
            if (DriverStation.isAutonomous()){
                SwerveController.getInstance().setChannel("Auto");
                SwerveController.getInstance().setControl(new SwerveInput(), "Auto");
            }
            else{
                SwerveController.getInstance().setChannel("Driver");
                SwerveController.getInstance().setControl(new SwerveInput(), "Driver");
            }
        });
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
                close(),
                Commands.runOnce(() -> Swerve.getInstance().resetModulesToAbsolute())
        );
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        SwerveController.getInstance().periodic();

        double accLimitAt0 = 65;
        double accLimitAt10 = 24;
        double elevatorHeight = RobotContainer.getElevator().getHeight();
        double accLimit = (accLimitAt0 - accLimitAt10) / -10 * elevatorHeight + accLimitAt0;
        Constants.Swerve.kSwerveConstants.limits.maxSkidAcceleration = accLimit;

        if (driveToCoralCommand != null)
            Logger.recordOutput("Swerve/Coral Command", driveToCoralCommand.isScheduled() && !driveToCoralCommand.isFinished());
        if (driveToReefCommand != null)
            Logger.recordOutput("Swerve/Reef Command", driveToReefCommand.isScheduled() && !driveToReefCommand.isFinished());
        Logger.recordOutput("Swerve/Reef Target", target);
        Logger.recordOutput("Swerve/Acceleration Limit", accLimit);
    }
}