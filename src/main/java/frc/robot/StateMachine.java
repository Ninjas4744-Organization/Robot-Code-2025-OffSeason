package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;

import java.util.Set;

public class StateMachine extends StateMachineBase<States> {
    public static StateMachine getInstance(){
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected boolean canChangeRobotState(States currentState, States wantedState) {
        return switch (currentState) {
            case IDLE -> Set.of(
                    States.INTAKE_CORAL,
                    States.INTAKE_ALGAE_HIGH,
                    States.INTAKE_ALGAE_LOW
            ).contains(wantedState);

                
            case INTAKE_CORAL -> Set.of(
                    States.CORAL_IN_INTAKE
            ).contains(wantedState);
            case CORAL_IN_INTAKE -> Set.of(
                    States.PREPARE_CORAL_OUTTAKE_LOW,
                    States.ARM_INTAKE
            ).contains(wantedState);

            // LOW CORAL
            case PREPARE_CORAL_OUTTAKE_LOW -> Set.of(
                    States.CORAL_OUTTAKE_LOW
            ).contains(wantedState);
            case CORAL_OUTTAKE_LOW -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);

            // HIGH CORAL
            case ARM_INTAKE -> Set.of(
                    States.CORAL_IN_ARM
            ).contains(wantedState);
            case CORAL_IN_ARM -> Set.of(
                    States.PREPARE_CORAL_OUTTAKE_HIGH
            ).contains(wantedState);
            case PREPARE_CORAL_OUTTAKE_HIGH -> Set.of(
                    States.CORAL_OUTTAKE_HIGH
            ).contains(wantedState);
            case CORAL_OUTTAKE_HIGH -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);


            // ALGAE
            case INTAKE_ALGAE_LOW,INTAKE_ALGAE_HIGH -> Set.of(
                    States.ALGAE_IN_ARM
            ).contains(wantedState);
            case ALGAE_IN_ARM -> Set.of(
                    States.PREPARE_ALGAE_OUTTAKE
            ).contains(wantedState);
            case PREPARE_ALGAE_OUTTAKE -> Set.of(
                    States.ALGAE_OUTTAKE
            ).contains(wantedState);
            case ALGAE_OUTTAKE -> Set.of(
                    States.CLOSE,
                    States.RESET
            ).contains(wantedState);


            case CLOSE,RESET -> Set.of(
                    States.IDLE
            ).contains(wantedState);
        };
    }

    @Override
    protected void setCommandMap() {
        //region idle
        addCommand(States.IDLE, Commands.none());
        //endregion

        //region intake coral
        addCommand(States.INTAKE_CORAL,Commands.none() );
        addCommand(States.CORAL_IN_INTAKE, Commands.none());
        //endregion

        //region outtake coral
        addCommand(States.PREPARE_CORAL_OUTTAKE_LOW, Commands.sequence(
                intakeAngle.setPercent(() -> 0.5)
                //TODO: implement angle check
//                Commands.waitUntil(() -> {
//                    RobotContainer.getIntakeAngle()
//                }),
        ));
        addCommand(States.CORAL_OUTTAKE_LOW, Commands.sequence(
                intake.outputCoral(),
                Commands.waitSeconds(0.2),
                intakeAngle.setPercent(() -> 0),
                Commands.waitSeconds(0.2),
                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));
        addCommand(States.ARM_INTAKE, Commands.sequence(
                 //TODO: implement angle check,
                outtake.intakeCoral(),
                Commands.waitUntil(arm::isCoralInside),
                Commands.runOnce(()-> changeRobotState(States.CORAL_IN_ARM))
        ));
        addCommand(States.CORAL_IN_ARM, Commands.none());
        addCommand(States.PREPARE_CORAL_OUTTAKE_HIGH, Commands.none());
        addCommand(States.CORAL_OUTTAKE_HIGH, Commands.none());
        //endregion

        //region intake algae
        addCommand(States.INTAKE_ALGAE_LOW, Commands.sequence(
                arm.setAngle(Rotation2d.kZero),
                Commands.waitUntil(arm::atGoal),
                outtake.intakeCoral(),
                Commands.waitUntil(arm::isCoralInside),
                Commands.runOnce(()-> changeRobotState(States.ALGAE_IN_ARM))
                // No need to stop the intaking because we want to keep ahold of the algae. we'll stop only when we outtake.
        ));
        addCommand(States.INTAKE_ALGAE_HIGH, Commands.sequence(
                Commands.runOnce(()-> changeRobotState(States.ALGAE_IN_ARM))
                // No need to stop the intaking because we want to keep ahold of the algae. we'll stop only when we outtake.
        ));
        addCommand(States.ALGAE_IN_ARM, Commands.none());
        //endregion

        //region outtake algae
        addCommand(States.PREPARE_ALGAE_OUTTAKE, Commands.none());
        addCommand(States.ALGAE_OUTTAKE, Commands.sequence(
                outtake.outtakeAlgae(),
                Commands.waitSeconds(0.2),
                outtake.stop(),
                Commands.runOnce(()-> changeRobotState(States.CLOSE))
        ));
        //endregion

        //region close + reset
        addCommand(States.CLOSE, Commands.sequence(
                Commands.parallel(
                        arm.setAngle(Rotation2d.kZero),
                        elevator.setHeight(() -> 0),
                        //TODO: implement angle check,
                        intake.setPercent(()->0)
                ),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal()),
                Commands.runOnce(()-> changeRobotState(States.IDLE))
        ));
        addCommand(States.RESET, Commands.sequence(
                elevator.reset(),
                arm.reset(),
                Commands.waitUntil(() -> elevator.isReset() && arm.isReset()),
                Commands.runOnce(()-> changeRobotState(States.IDLE))
        ));
        //endregion
    }
}
