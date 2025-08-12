package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
    private OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    private boolean enabled;

    public Outtake(boolean enabled, OuttakeIO io) {
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
        Logger.processInputs("Outtake", inputs);

    }

    //--Commands
    public Command stop(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.getController().setPercent(0));
    }

    public Command intakeObject(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.getController().setPercent(Constants.OuttakeSpeeds.Intake.get()));
    }

    public Command outtakeCoral(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.getController().setPercent(Constants.OuttakeSpeeds.OuttakeCoral.get()));
    }

    public Command outtakeAlgae(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.getController().setPercent(Constants.OuttakeSpeeds.OuttakeAlgae.get()));
    }
}