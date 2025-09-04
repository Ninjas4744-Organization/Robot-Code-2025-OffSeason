package frc.robot.subsystems.intakeangle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class IntakeAngle extends SubsystemBase {
    private IntakeAngleIO io;
    private final IntakeAngleIOInputsAutoLogged inputs = new IntakeAngleIOInputsAutoLogged();
    private boolean enabled;

    public IntakeAngle(boolean enabled, IntakeAngleIO io) {
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
        Logger.processInputs("Intake Angle", inputs);
    }

    public Command setPercent(DoubleSupplier percent) {
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(
            () -> io.setPercent(percent.getAsDouble())
        );
    }

    public Command setAngle(Rotation2d angle) {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(
            () -> io.setPosition(angle.getRadians())
        );
    }

    public Rotation2d getAngle(){
        if (!enabled) {
            return Rotation2d.kZero;
        }
        return Rotation2d.fromRadians(inputs.Position);
    }

    public Command lookDown() {
        return setAngle(Rotation2d.fromDegrees(Constants.IntakeAnglePositions.LOOK_DOWN.get()));
    }

    public Command lookAtL1() {
        return setAngle(Rotation2d.fromDegrees(Constants.IntakeAnglePositions.LOOK_AT_L1.get()));
    }

    public Command lookAtArm() {
        return setAngle(Rotation2d.fromDegrees(Constants.IntakeAnglePositions.LOOK_AT_ARM.get()));
    }

    public boolean atGoal(){
        if (!enabled){
            return true;
        }
        return inputs.AtGoal;
    }

    public Command reset() {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.run(() -> {
            io.setPercent(-0.2);
        }).until(() -> inputs.LimitSwitch);
    }

    public boolean isReset() {
        if (!enabled) {
            return true;
        }
        return inputs.LimitSwitch;
    }

    public boolean isEnabled() {
        return enabled;
    }
}