package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private static Elevator instance;
    private boolean enabled;

    public static Elevator getInstance() {
        return instance;
    }

    public static void createInstance(Elevator elevator) {
        instance = elevator;
    }

    public Elevator(boolean enabled, ElevatorIO io) {
        if (enabled) {
            this.io = io;
            io.setup();
        }
        this.enabled = enabled;
    }

    public ElevatorIO getIO() {
        if (enabled)
            return io;
        else
            return new ElevatorIO() {
            };
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
}