package frc.robot;

public enum States {
    IDLE,

    INTAKE_CORAL,
    CORAL_IN_INTAKE, //the coral is set in the middle of the intake

    PREPARE_CORAL_OUTTAKE_L1, //For L1
    CORAL_OUTTAKE_L1,

    TRANSFER_CORAL_FROM_INTAKE_TO_OUTTAKE, //transfer the coral from the intake from the bottom to the arm
    CORAL_IN_OUTTAKE,//the coral is set in the arm
    TRANSFER_CORAL_FROM_OUTTAKE_TO_INTAKE, //transfer the coral from the arm back to the intake (

    // automatic driving for when we want to score a coral NOTE:(FOR ALL L LEVELS)
    DRIVE_LEFT_REEF,
    DRIVE_RIGHT_REEF,
    PREPARE_CORAL_OUTTAKE, //For L2-L4
    CORAL_OUTTAKE,

    INTAKE_ALGAE_LOW, //moving the arm and grabbing algae from the floor
    INTAKE_ALGAE_HIGH, //moving the arm and grabbing algae from the reef
    ALGAE_IN_OUTTAKE, //Finished the initial grab of the algae
    PREPARE_ALGAE_OUTTAKE, //towards the barge
    ALGAE_OUTTAKE,

    CLOSE,
    RESET,

    PREPARE_CLIMB,
    CLIMB,
    CLIMBED
}
