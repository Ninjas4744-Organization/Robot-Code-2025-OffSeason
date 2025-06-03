// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.commands.nRepeatingSequenceCommand;
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

    public RobotContainer() {
//        autoChooser = AutoBuilder.buildAutoChooser();

        SwerveSubsystem.createInstance(new SwerveSubsystem(true));

        RobotStateWithSwerve.setInstance(new RobotState(Constants.kSwerveConstants.kinematics, Constants.kInvertGyro, v -> new double[3], 45));
        RobotState.getInstance().setRobotState(States.SIGMA);

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
        driverController.cross().onTrue(new nRepeatingSequenceCommand(() -> 5, Commands.print("a"), Commands.print("b")));

        SwerveSubsystem.getInstance().setDefaultCommand(Commands.run(() -> {
            SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                    new ChassisSpeeds(-MathUtil.applyDeadband(driverController.getLeftY(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                            -MathUtil.applyDeadband(driverController.getLeftX(), Constants.kJoystickDeadband) * Constants.kDriverSpeedFactor,
                            -MathUtil.applyDeadband(driverController.getRightX(), Constants.kJoystickDeadband) * Constants.kDriverRotationSpeedFactor
                    )), Constants.kDriverFieldRelative, "Driver");
        }, SwerveSubsystem.getInstance()));
    }

    public void periodic() {
        if (Robot.isSimulation()) {
            Logger.recordOutput("Simulation Field/Corals", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
            SimulatedArena.getInstance().simulationPeriodic();
        }
    }

    public Command getAutonomousCommand() {
        return Commands.none();
//        return autoChooser.getSelected();
    }

    public void reset() {
        Swerve.getInstance().resetModulesToAbsolute();
    }
}
