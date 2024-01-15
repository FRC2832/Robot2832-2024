package org.livoniawarriors.leds;

import org.livoniawarriors.ColorHSV;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class shows a slow breathing pattern on the leds, slowly turning them on and off
 */
public class BreathLeds extends Command {
    final double STEP_VALUE = 5;  //how much to increment value every 20ms
    final double MAX_VALUE = 200;
    final double MIN_VALUE = 25;

    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    int hue, sat, breath;
    boolean increment;

    public BreathLeds(ILedSubsystem leds, Color color) {
        this.leds = leds;
        ColorHSV hsv = ColorHSV.fromColor(color);
        hue = (int)hsv.hue;
        sat = (int)hsv.sat;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
        increment = true;
        breath = 125;
    }

    @Override
    public void execute() {
        if(increment) {
            breath += STEP_VALUE;
            if(breath >= MAX_VALUE) {
                increment = false;
            }
        } else {
            breath -= STEP_VALUE;
            if(breath <= MIN_VALUE) {
                increment = true;
            }
        }

        //set the pattern
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, hue, sat, breath);
        }
        leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { }
}
