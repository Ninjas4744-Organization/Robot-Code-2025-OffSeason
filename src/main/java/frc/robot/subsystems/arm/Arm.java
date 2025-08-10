package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.kArmCanCoderReversed;

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

        canCoder = new CANcoder(Constants.kArmCanCoderID);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Constants.kArmCanCoderOffset;
        config.MagnetSensor.SensorDirection = kArmCanCoderReversed;
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
        return Commands.runOnce(() -> {io.getController().setPosition(angle.getRadians());});
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

    public Rotation2d getCANCoder(){
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    public Command reset(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> {io.getController().setEncoder(getCANCoder().getRadians());}).andThen(setAngle(Rotation2d.fromDegrees(Constants.ArmPositions.Close.get())));
    }

    public boolean isReset(){
       if (!enabled){
            return true;
       }
       return Math.abs(getCANCoder().getDegrees()) < (Constants.ArmPositions.Close.get() + Constants.kArmResetTolerance);
    }
}