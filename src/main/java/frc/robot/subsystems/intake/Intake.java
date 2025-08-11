package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean enabled;

    public Intake(boolean enabled, IntakeIO io) {
        if (enabled) {
            this.io = io;
            io.setup();
        }
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command setPercent(DoubleSupplier percent) {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(
            () -> io.getController().setPercent(percent.getAsDouble())
        );
    }

    public boolean isCoralInside() {
        if (!enabled) {
            return true;
        }
        return RobotState.isCoralInIntake();
    }

    public Command outputCoral() {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(
            () -> io.getController().setPercent(-0.8)
        );
    }
}