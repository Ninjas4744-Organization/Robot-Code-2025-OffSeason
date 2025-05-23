// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.Swerve;
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

        RobotStateWithSwerve.setInstance(new RobotState(Constants.kSwerveConstants.kinematics, Constants.kInvertGyro, v -> new double[3], 0));
        RobotState.getInstance().setRobotState(States.SIGMA);

        SwerveSubsystem.createInstance(new SwerveSubsystem(true));

        switch (Constants.kCurrentMode) {
            case REAL, SIM:
                Arm.createInstance(new Arm(true, new ArmIOController()));
                Elevator.createInstance(new Elevator(false, new ElevatorIOController()));
                break;

            case REPLAY:
                Arm.createInstance(new Arm(true, new ArmIO() {
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
        driverController.cross().toggleOnTrue(
            Commands.startEnd(
                () -> Arm.getInstance().getIO().setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Open.get())),
                () -> Arm.getInstance().getIO().setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Close.get()))
            )
        );

        driverController.circle().toggleOnTrue(
            Commands.startEnd(
                () -> Arm.getInstance().getIO().setPercent(1),
                () -> Arm.getInstance().getIO().setPercent(0)
            )
        );

        SwerveSubsystem.getInstance().setDefaultCommand(Commands.run(() -> {
            Swerve.getInstance().drive(new ChassisSpeeds(-driverController.getLeftY() * 5, -driverController.getLeftX() * 5, -driverController.getRightX() * 10), true);
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
}
