package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private boolean enabled;

    public Arm(boolean enabled, ArmIO io) {
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
        Logger.processInputs("Arm", inputs);
    }

    public Command setAngle(Rotation2d angle){
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(() -> io.setPosition(angle));
    }

    public Command lookDown() {
        return setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Close.get()));
    }

    public Command lookAtCoralReef(int L) {
        return switch (L) {
            case 2 -> setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.L4.get()));
            case 3 -> setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.L3.get()));
            case 4 -> setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.L2.get()));
            default -> setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Close.get()));
        };
    }

    public Command lookAtAlgaeReef() {
        return setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.HighAlgaeOut.get()));
    }

    public Command lookAtAlgaeFloor() {
        return setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.LowAlgaeOut.get()));
    }

    public Command lookAtBarge() {
        return setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Net.get()));
    }

    public Rotation2d getAngle(){
        if (!enabled) {
            return Rotation2d.kZero;
        }
        return Rotation2d.fromRadians(inputs.Position);
    }

    public boolean atGoal(){
        if (!enabled){
            return true;
        }
        return inputs.AtGoal;
    }

    public Command reset(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.setEncoder(inputs.AbsoluteAngle.getRadians())).andThen(setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Close.get())));
    }

    public boolean isReset(){
       if (!enabled){
            return true;
       }
       return inputs.AtGoal;
    }

    public boolean isEnabled() {
        return enabled;
    }
}