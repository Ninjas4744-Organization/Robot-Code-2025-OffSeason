package frc.robot;

import frc.lib.NinjasLib.statemachine.StateMachineBase;

public class StateMachine extends StateMachineBase<States> {
    public static StateMachine getInstance(){
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected boolean canChangeRobotState(States currentState, States wantedState) {
        return switch (currentState) {
            case IDLE -> false;
            case INTAKE_CORAL -> false;
            case CORAL_IN_INTAKE -> false;
            case PREPARE_CORAL_OUTTAKE_LOW -> false;
            case CORAL_OUTTAKE_LOW -> false;
            case ARM_INTAKE -> false;
            case CORAL_IN_ARM -> false;
            case PREPARE_CORAL_OUTTAKE_HIGH -> false;
            case CORAL_OUTTAKE_HIGH -> false;
            case PREPARE_INTAKE_ALGAE_HIGH -> false;
            case PREPARE_INTAKE_ALGAE_LOW -> false;
            case INTAKE_ALGAE -> false;
            case PREPARE_ALGAE_OUTTAKE -> false;
            case ALGAE_OUTTAKE -> false;
            case CLOSE -> false;
            case RESET -> false;
        };
    }

    @Override
    protected void setCommandMap() {

    }
}
