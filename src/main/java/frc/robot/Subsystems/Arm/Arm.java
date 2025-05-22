package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;
    private boolean enabled;

    public static Arm getInstance() {
        return instance;
    }

    public static void createInstance(Arm arm) {
        instance = arm;
    }

    public Arm(boolean enabled, ArmIO io) {
        if (enabled)
            this.io = io;
        this.enabled = enabled;
    }

    public ArmIO getIO() {
        if (enabled)
            return io;
        else
            return new ArmIO() {
            };
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }
}