package frc.robot.loggedcontroller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoggedCommandController {
    private LoggedCommandControllerIO.LoggedCommandControllerIOInputs inputs = new LoggedCommandControllerIOInputsAutoLogged();
    private LoggedCommandControllerIO io;

    public LoggedCommandController() {

    }

    public Trigger cross() {
        return new Trigger(() -> inputs.cross);
    }
}
