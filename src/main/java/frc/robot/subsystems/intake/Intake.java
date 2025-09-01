package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean enabled;
    private boolean isCoralInside = false;

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

        System.out.println(Math.abs(io.getController().getCurrent()) + ", " + io.getController().getOutput());
        if (Math.abs(io.getController().getCurrent()) > 35 && io.getController().getOutput() < 0)
            isCoralInside = true;

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

//        return RobotState.isCoralInIntake();
        return isCoralInside;
    }

    public Command intake() {
        return setPercent(Constants.IntakeSpeeds.Intake::get);
    }

    public Command outtake() {
        return Commands.sequence(
                Commands.runOnce(() -> isCoralInside = false),
                setPercent(Constants.IntakeSpeeds.Outtake::get)
        );
    }

    public Command stop() {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(
            () -> io.getController().setPercent(0)
        );
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
                Commands.runOnce(() -> isCoralInside = false),
                intake(),
                Commands.race(
                        Commands.waitUntil(() -> Math.abs(io.getController().getCurrent()) > 35),
                        Commands.waitSeconds(0.25)
                ),
                stop()
        );
    }

    public boolean isEnabled() {
        return enabled;
    }
}