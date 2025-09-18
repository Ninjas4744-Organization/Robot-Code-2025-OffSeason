package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.States;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
    private OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    private boolean enabled;
    private boolean isCoralInside = false;
    private boolean isAlgaeInside = false;
    private Timer currentTimer = new Timer();

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

        if (Math.abs(inputs.Current) > Constants.kOuttakeCurrentThreshold && inputs.Output < 0) {
            if (!currentTimer.isRunning())
                currentTimer.restart();
        } else {
            currentTimer.stop();
            currentTimer.reset();
        }

        if(currentTimer.get() > 0.25){
            if (RobotState.getInstance().getRobotState() == States.INTAKE_CORAL)
                isCoralInside = true;
            else if (RobotState.getInstance().getRobotState() == States.INTAKE_ALGAE_HIGH || RobotState.getInstance().getRobotState() == States.INTAKE_ALGAE_LOW)
                isAlgaeInside = true;
        }

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Outtake", inputs);
    }

    public Command stop(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.setPercent(0));
    }

    public Command intake() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(Constants.OuttakeSpeeds.Intake.get()));
    }

    public Command outtake() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            io.setPercent(Constants.OuttakeSpeeds.Outtake.get());
            isAlgaeInside = false;
            isCoralInside = false;
        });
    }

    public boolean isCoralInside() {
        if (!enabled)
            return false;

        return isCoralInside;
    }

    public boolean isAlgaeInside() {
        if (!enabled)
            return false;

        return isAlgaeInside;
    }

    private boolean hadObjectInside = false;

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
                intake(),
                Commands.race(
                        Commands.waitUntil(() -> {
                            if (Math.abs(inputs.Current) > Constants.kOuttakeCurrentThreshold) {
                                if (!currentTimer.isRunning())
                                    currentTimer.restart();
                            } else {
                                currentTimer.stop();
                                currentTimer.reset();
                            }

                            if(currentTimer.get() > 0.25)
                                hadObjectInside = true;
                            return hadObjectInside;
                        }),
                        Commands.waitSeconds(0.5)
                ),
                stop(),
                Commands.runOnce(() -> {
                    if (!hadObjectInside) {
                        isCoralInside = false;
                        isAlgaeInside = false;
                    } else if (!isCoralInside && !isAlgaeInside) {
                        Command outtake = outtake().andThen(Commands.waitSeconds(0.5)).andThen(stop());
                        outtake.schedule();
                    }
                })
        );
    }
}