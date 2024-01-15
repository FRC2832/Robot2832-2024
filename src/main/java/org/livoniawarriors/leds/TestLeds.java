package org.livoniawarriors.leds;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class TestLeds extends Command {
    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;

    IntegerSubscriber subIndex;
    IntegerSubscriber subHue;
    IntegerSubscriber subSat;
    IntegerSubscriber subValue;

    public TestLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() { return true; }

    @Override
    public void initialize() { 
        //normally this should be in the constructor, but moving to init so it won't always be in NT
        subIndex = UtilFunctions.getNtSub("/TestLeds/Index", -1);
        subHue = UtilFunctions.getNtSub("/TestLeds/Hue", 0);
        subSat = UtilFunctions.getNtSub("/TestLeds/Saturation", 20);
        subValue = UtilFunctions.getNtSub("/TestLeds/Value", 0);
    }

    @Override
    public void execute() {
        //get color
        Color newColor = Color.fromHSV((int)subHue.get(), (int)subSat.get(), (int)subValue.get());
        int pos = (int)subValue.get();

        //check if we are doing individual index
        if(pos >= 0){
            //fill the whole string with black
            for(int i=0; i<m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, Color.kBlack);
            }
            //color the specific LED
            int max = leds.getLength()-1;
            if(pos > max)  pos = max;
            m_ledBuffer.setLED(pos, newColor);
        } else {
            //fill the whole string with the color
            for(int i=0; i<m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setLED(i, newColor);
            }
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
