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
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.*;

import java.util.List;
import java.util.function.BooleanSupplier;

public class SwerveSubsystem extends SubsystemBase {
    private boolean enabled;
    private List<Pose2d> reefAprilTags;
    private Pose2d target;
    private ProfiledPIDController pidRotation;
    private Command driveToReefCommand;
    private Command driveToCoralCommand;

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(Constants.kSwerveConstants));
            SwerveController.setInstance(new SwerveController(Constants.kSwerveControllerConstants));
            SwerveController.getInstance().setChannel("Driver");

            AprilTagFieldLayout layout = Constants.getFieldLayout();
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
            target = Pose2d.kZero;
            pidRotation = new ProfiledPIDController(3, 0, 0, new TrapezoidProfile.Constraints(Constants.kSwerveConstants.maxAngularVelocity, Constants.kSwerveConstants.maxAcceleration));
        }
    }

    public void swerveDrive(LoggedCommandController controller) {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                new SwerveInput(-MathUtil.applyDeadband(controller.getLeftY(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(controller.getLeftX(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(controller.getRightX(), Constants.kJoystickDeadband) * Constants.kDriverRotationSpeedFactor,
                        Constants.kDriverFieldRelative
                )), "Driver");
    }

    double f(double error){
        double a = 1;
        double b = 2;
        return Math.pow(a * error, 1 / b);
    }

    public Command autoDriveToReef(BooleanSupplier isRightSide){
        driveToReefCommand = Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("AutoReef");
                target = RobotState.getInstance().getRobotPose().nearest(reefAprilTags);
                target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));
                target = target.transformBy(new Transform2d(-Constants.kAutoDriveDistFromReef, isRightSide.getAsBoolean() ? -Constants.kAutoDriveRightSideOffset : Constants.kAutoDriveLeftSideOffset, Rotation2d.kZero));

                pidRotation.reset(RobotState.getInstance().getRobotPose().getRotation().getRadians());
            }),

            Commands.run(() -> {
                Translation2d translation = RobotState.getInstance().getTranslation(target);
                Translation2d dir = translation.div(translation.getNorm());
                double velocity = f(translation.getNorm());

                SwerveController.getInstance().setControl(
                    new SwerveInput(
                        velocity * dir.getX(),
                        velocity * dir.getY(),
                        pidRotation.calculate(RobotState.getInstance().getRobotPose().getRotation().getRadians(), target.getRotation().getRadians()),
                        true
                    ), "AutoReef"
                );
            }).until(this::atGoal)
        );
        return driveToReefCommand;
    }

    private Timer noCoralTimer = new Timer();
    public Command driveToCoral() {
        driveToCoralCommand = Commands.sequence(
                Commands.waitUntil(() -> CoralDetection.getInstance().hasTarget()),
                Commands.runOnce(() -> {
                    SwerveController.getInstance().setChannel("AutoCoral");
                    StateMachine.getInstance().changeRobotState(States.INTAKE_CORAL);
                }),
                Commands.run(() -> {
                    Translation2d dir = CoralDetection.getInstance().getFieldRelativeDir();

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

                    if (!CoralDetection.getInstance().hasTarget() && !noCoralTimer.isRunning())
                        noCoralTimer.restart();

                    if (CoralDetection.getInstance().hasTarget()) {
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
            driveToReefCommand.cancel(); //TODO: Check if the command works even if we don't create the command each time but reuse the same one

        if (driveToCoralCommand != null)
            driveToCoralCommand.cancel();
    }

    public boolean atGoal() {
        return Math.abs(RobotState.getInstance().getDistance(target))  < Constants.kAutoDriveDistThreshold;
    }

    public Command close() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
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
        if (enabled)
            SwerveController.getInstance().periodic();
    }
}