package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private static Arm instance;

    public static Arm getInstance() {
        if (instance == null)
            instance = new Arm();

        return instance;
    }

    public Arm() {
        io = new ArmIOController();
    }

    public void setAngle(Rotation2d angle) {
        io.setAngle(angle);
    }

    public void setPercent(double percent) {
        io.setPercent(percent);
    }

    @Override
    public void periodic() {
        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }
}