package frc.robot;

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

    }
}
