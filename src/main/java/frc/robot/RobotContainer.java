package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private CommandPS5Controller driverController;
    private CommandPS5Controller operatorController;

    private static Elevator elevator;
    private static Arm arm;
    private static SwerveSubsystem swerveSubsystem;

    private SendableChooser<Command> autoChooser;
    private FOMCalculator fomCalculator;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL, SIM:
                arm = new Arm(false, new ArmIOController());
                elevator = new Elevator(false, new ElevatorIOController());
                swerveSubsystem = new SwerveSubsystem(true);
                break;

            case REPLAY:
                arm = new Arm(false, new ArmIO() {
                });
                elevator = new Elevator(false, new ElevatorIO() {
                });
                break;
        }

        RobotStateBase.setInstance(new RobotState(Constants.kSwerveConstants.kinematics, Constants.kInvertGyro, Constants.kPigeonID, Constants.kSwerveConstants.enableOdometryThread));

        Vision.setConstants(Constants.kVisionConstants);
        fomCalculator = new FOMCalculator();

//        autoChooser = AutoBuilder.buildAutoChooser();

        driverController = new CommandPS5Controller(Constants.kDriverControllerPort);
        operatorController = new CommandPS5Controller(Constants.kOperatorControllerPort);

        if (Robot.isSimulation()) {
            for (int i = 0; i < 10; i++)
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1.5, 4, Rotation2d.kZero)));
        }

        configureBindings();
    }

    private void configureBindings() {
        driverController.L2().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.L1().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(RobotState.getInstance().getRobotPose().getRotation())));
    }

    public static Elevator getElevator() {
        return elevator;
    }

    public static Arm getArm() {
        return arm;
    }

    private void swerveDrive() {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
            new SwerveInput(-MathUtil.applyDeadband(driverController.getLeftY(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                -MathUtil.applyDeadband(driverController.getRightX(), Constants.kJoystickDeadband) * Constants.kDriverRotationSpeedFactor,
                Constants.kDriverFieldRelative
            )), "Driver");
    }

    public void periodic() {
        swerveDrive();

        if (Robot.isSimulation()) {
            Logger.recordOutput("Simulation Field/Corals", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            SimulatedArena.getInstance().simulationPeriodic();
        }

        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
        fomCalculator.update(estimations);
        for (int i = 0; i < estimations.length; i++)
            RobotState.getInstance().updateRobotPose(estimations[i], fomCalculator.getOdometryFOM(), fomCalculator.getVisionFOM()[i]);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
//        return autoChooser.getSelected();
    }

    public void reset() {
        Swerve.getInstance().resetModulesToAbsolute();
        SwerveController.getInstance().setChannel("Driver");
        SwerveController.getInstance().setControl(new SwerveInput(), "Driver");
    }
}
