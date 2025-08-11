package frc.robot;

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
        addCommand(States.PREPARE_CORAL_OUTTAKE_LOW, Commands.none());
        addCommand(States.CORAL_OUTTAKE_LOW, Commands.none());
        addCommand(States.ARM_INTAKE, Commands.none());
        addCommand(States.CORAL_IN_ARM, Commands.none());
        addCommand(States.PREPARE_CORAL_OUTTAKE_HIGH, Commands.none());
        addCommand(States.CORAL_OUTTAKE_HIGH, Commands.none());
        //endregion

        //region intake algae
        addCommand(States.INTAKE_ALGAE_LOW, Commands.none());
        addCommand(States.INTAKE_ALGAE_HIGH, Commands.none());
        addCommand(States.ALGAE_IN_ARM, Commands.none());
        //endregion

        //region outtake algae
        addCommand(States.PREPARE_ALGAE_OUTTAKE, Commands.none());
        addCommand(States.ALGAE_OUTTAKE, Commands.none());
        //endregion

        //region close + reset
        addCommand(States.CLOSE, Commands.none());
        addCommand(States.RESET, Commands.none());
        //endregion
    }
}
