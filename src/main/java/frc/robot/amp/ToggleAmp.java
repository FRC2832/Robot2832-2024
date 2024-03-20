package frc.robot.amp;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleAmp extends Command {
    private Amp amp;

    public ToggleAmp(Amp amp) {
        this.amp = amp;
    }

    @Override
    public void execute() {
        amp.ToggleAmp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
