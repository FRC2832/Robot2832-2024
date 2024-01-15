package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class FillLeds extends Command {
    final double STEP_TIME = 0.04;  //increment every 40ms
    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    double startTime;
    int currentLight;
    Color color;

    public FillLeds(ILedSubsystem leds, Color color) {
        this.leds = leds;
        this.color = color;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        //calculate the current light
        double time = Timer.getFPGATimestamp() - startTime;
        currentLight = (int)(time/STEP_TIME);

        //set the pattern
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
            if(i > currentLight) {
                color = Color.kBlack;
            }
            m_ledBuffer.setLED(i, color);
        }
        leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return currentLight > m_ledBuffer.getLength();
    }

    @Override
    public void end(boolean interrupted) { }
}
