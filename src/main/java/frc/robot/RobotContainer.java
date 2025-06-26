package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.dataclasses.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.vision.Vision;
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
    private SendableChooser<Command> autoChooser;
    private CommandPS5Controller driverController;
    private CommandPS5Controller operatorController;
    private FOMCalculator fomCalculator;
    private BuiltInAccelerometer roborioAccelerometer;

    public RobotContainer() {
//        autoChooser = AutoBuilder.buildAutoChooser();

        roborioAccelerometer = new BuiltInAccelerometer();

        SwerveSubsystem.createInstance(new SwerveSubsystem(true));
        RobotStateWithSwerve.setInstance(new RobotState(Constants.kSwerveConstants.kinematics, Constants.kInvertGyro, 45));
        RobotState.getInstance().setRobotState(States.SIGMA);

        Vision.setConstants(Constants.kVisionConstants);
        fomCalculator = new FOMCalculator();

        switch (Constants.kCurrentMode) {
            case REAL, SIM:
                Arm.createInstance(new Arm(false, new ArmIOController()));
                Elevator.createInstance(new Elevator(false, new ElevatorIOController()));
                break;

            case REPLAY:
                Arm.createInstance(new Arm(false, new ArmIO() {
                }));
                Elevator.createInstance(new Elevator(false, new ElevatorIO() {
                }));
                break;
        }

        driverController = new CommandPS5Controller(Constants.kDriverControllerPort);
        operatorController = new CommandPS5Controller(Constants.kOperatorControllerPort);

        if (Robot.isSimulation()) {
            for (int i = 0; i < 10; i++)
                SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1.5, 4, Rotation2d.kZero)));
        }

        configureBindings();
    }

    private void configureBindings() {
        SwerveSubsystem.getInstance().setDefaultCommand(Commands.run(() -> {
            SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                    new ChassisSpeeds(-MathUtil.applyDeadband(driverController.getLeftY(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                            -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                            -MathUtil.applyDeadband(driverController.getRightX(), Constants.kJoystickDeadband) * Constants.kDriverRotationSpeedFactor
                    )), Constants.kDriverFieldRelative, "Driver");
        }, SwerveSubsystem.getInstance()));

        driverController.L2().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
    }

    public void periodic() {
        if (Robot.isSimulation()) {
            Logger.recordOutput("Simulation Field/Corals", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            SimulatedArena.getInstance().simulationPeriodic();
        }

        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
        fomCalculator.update(estimations);
        for (int i = 0; i < estimations.length; i++)
            RobotState.getInstance().updateRobotPose(estimations[i], fomCalculator.getOdometryFOM(), fomCalculator.getVisionFOM()[i]);

        Logger.recordOutput("Robot Acceleration", new Translation2d(roborioAccelerometer.getX() * 9.81, roborioAccelerometer.getY() * 9.81).getNorm());
    }

    public Command getAutonomousCommand() {
        return Commands.none();
//        return autoChooser.getSelected();
    }

    public void reset() {
        Swerve.getInstance().resetModulesToAbsolute();
        SwerveController.getInstance().setChannel("Driver");
        SwerveController.getInstance().setControl(new ChassisSpeeds(), false, "Driver");
    }
}
