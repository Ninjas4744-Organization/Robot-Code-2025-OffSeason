// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private CommandPS5Controller driverController;
    private CommandPS5Controller operatorController;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();

        driverController = new CommandPS5Controller(Constants.kDriverControllerPort);
        operatorController = new CommandPS5Controller(Constants.kOperatorControllerPort);

        configureBindings();
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
