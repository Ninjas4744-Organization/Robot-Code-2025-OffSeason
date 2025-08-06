package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private boolean enabled;

    public Climber(boolean enabled, ClimberIO io) {
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
        Logger.processInputs("Climber", inputs);
    }

    public Command setPercent(double percent) {
        return Commands.runOnce(
            () -> io.getController().setPercent(percent)
        );
    }

    //fixme: needs review
    public boolean isAttachedToCage() {
        return Constants.kClimberControllerConstants.real.currentLimit - io.getController().getCurrent() <= 0.02 ;
    }


}