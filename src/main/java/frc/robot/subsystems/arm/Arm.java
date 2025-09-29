package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

        if (isArmUnsafe())
            io.stop();

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    public boolean isArmUnsafe() {
        boolean isIntakeClosed = RobotContainer.getIntakeAngle().getAngle().getDegrees() > 45;
        double angle = MathUtil.inputModulus(Units.radiansToDegrees(inputs.Goal), -180, 180);
        double elevatorHeight = RobotContainer.getElevator().getHeight();

        if (isIntakeClosed) {
            if (angle <= 0 && angle >= -180) {
                if (angle <= -45 && angle >= -135) {
                    return elevatorHeight < 0.4;
                } else {
                    return elevatorHeight < 0.7;
                }
            }
        } else {
            if (angle <= -45 && angle >= -135)
                return elevatorHeight < 0.5;
        }

        return false;
    }

    public Command setAngle(Rotation2d angle){
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(() -> io.setPosition(angle));
    }

    public Command lookDown() {
        return setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get()));
    }

    public Command lookAtCoralReef(int L) {
        return switch (L) {
            case 2 -> setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.L4.get()));
            case 3 -> setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.L3.get()));
            case 4 -> setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.L2.get()));
            default -> setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get()));
        };
    }

    public Command lookAtAlgaeReef() {
        return setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.HighAlgaeOut.get()));
    }

    public Command lookAtAlgaeFloor() {
        return setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.LowAlgaeOut.get()));
    }

    public Command lookAtBarge() {
        return setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.Net.get()));
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
        return Commands.runOnce(() -> io.setEncoder(inputs.AbsoluteAngle.getRotations())).andThen(setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get())));
    }

    public boolean isReset(){
       if (!enabled){
            return true;
       }
       return inputs.AtGoal;
    }
}