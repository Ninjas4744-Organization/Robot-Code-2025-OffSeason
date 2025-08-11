package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private boolean enabled;

    public Elevator(boolean enabled, ElevatorIO io) {
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
        Logger.processInputs("Elevator", inputs);
    }

    public Command setHeight(DoubleSupplier wantedHeight) {
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(() -> {
            io.getController().setPosition(wantedHeight.getAsDouble());
        });
    }

    public boolean atGoal() {
        if (!enabled) {
            return true;
        }
        return io.getController().atGoal();
    }

    public Command reset() {
        if (!enabled) {
            return Commands.none();
        }
        return Commands.run(() -> {
            io.getController().setPercent(-0.2);
        }).until(() -> io.getController().getLimit());
    }

    public boolean isReset() {
        if (!enabled) {
            return true;
        }
        return io.getController().getLimit();
    }
}