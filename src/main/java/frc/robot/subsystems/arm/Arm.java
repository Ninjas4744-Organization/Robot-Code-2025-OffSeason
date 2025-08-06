package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
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
    private CANcoder canCoder;


    public Arm(boolean enabled, ArmIO io) {
        if (enabled) {
            this.io = io;
            io.setup();
        }
        this.enabled = enabled;

        canCoder = new CANcoder(Constants.armCanCoderID);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Constants.armCanCoderOffset;
        canCoder.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    //--Commands

    public Command setAngle(Rotation2d angle){
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(() -> {io.getController().setPosition(angle.getDegrees());});
    }

    public Rotation2d getAngle(){
        if (!enabled) {
            return Rotation2d.kZero;
        }
        return Rotation2d.fromRadians(io.getController().getPosition());
    }

    public boolean atGoal(){
        if (!enabled){
            return true;
        }
        return io.getController().atGoal();
    }

    public double getCanCoderPositionRadians(){
        double positionRotations = canCoder.getAbsolutePosition().getValueAsDouble();
        return positionRotations * 2 * Math.PI;
    }

    public double getCanCoderPositionDegrees(){
        double positionRotations = canCoder.getAbsolutePosition().getValueAsDouble();
        return positionRotations * 360;
    }

    public Command reset(){
        if (!enabled || getCanCoderPositionDegrees() == -90){
            return Commands.none();
        }
        return Commands.runOnce(() -> {io.getController().setEncoder(getCanCoderPositionRadians());});
    }

    public boolean isReset(){
       if (!enabled){
            return true;
       }
       return getCanCoderPositionDegrees() == -90;
    }
}