package frc.robot.subsystems.intake_angle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
//import frc.robot.subsystems.intake.IntakeAngleIOInputsAutoLogged;
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
            () -> io.getController().setPercent(percent.getAsDouble())
        );
    }

    public Command setAngle(Rotation2d angle) {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(
            () -> io.getController().setPosition(angle.getRadians())
        );
    }

    public Rotation2d getAngle(){
        if (!enabled) {
            return Rotation2d.kZero;
        }
        return Rotation2d.fromRadians(io.getController().getPosition());
    }

    public Command lookDown() {
        return setAngle(Constants.anglesForIntakeAngle.get(Constants.intakeAnglePositions.LOOK_DOWN));
    }
    public Command lookAtL1() {
        return setAngle(Constants.anglesForIntakeAngle.get(Constants.intakeAnglePositions.LOOK_TO_L1));
    }
    public Command lookAtArm() {
        return setAngle(Constants.anglesForIntakeAngle.get(Constants.intakeAnglePositions.LOOK_TO_ARM));
    }

    public boolean atGoal(){
        if (!enabled){
            return true;
        }
        return io.getController().atGoal();
    }
}