// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private CommandPS5Controller driverController;
    private CommandPS5Controller operatorController;

    public RobotContainer() {
//        autoChooser = AutoBuilder.buildAutoChooser();
        Arm.createInstance(true);
        Elevator.createInstance(false);

        driverController = new CommandPS5Controller(Constants.kDriverControllerPort);
        operatorController = new CommandPS5Controller(Constants.kOperatorControllerPort);

        configureBindings();
    }

    private void configureBindings() {
        driverController.cross().toggleOnTrue(
            Commands.startEnd(
                () -> Arm.getInstance().setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Open.get())),
                () -> Arm.getInstance().setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Close.get()))
            )
        );

        driverController.circle().toggleOnTrue(
            Commands.startEnd(
                () -> Arm.getInstance().setPercent(1),
                () -> Arm.getInstance().setPercent(0)
            )
        );
    }

    public Command getAutonomousCommand() {
        return Commands.none();
//        return autoChooser.getSelected();
    }
}
