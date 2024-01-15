package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase implements ILedSubsystem {
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    /**
     * Setup the Led Hardware
     * @param channel What PWM channel the strip is connected to
     * @param length How long the LED string is
     */
    public LedSubsystem(int channel, int length) {
        super();
        //create the hardware device on the specified
        leds = new AddressableLED(channel);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(length);
        //start the strip
        leds.start();
    }

    @Override
    public void periodic() {
        //actually command the leds to show the pattern
        leds.setData(buffer);
    }
    
    @Override
    public int getLength() {
        return buffer.getLength();
    }

    @Override
    public Color getLed(int index) {
        return buffer.getLED(index);
    }

    @Override
    public void setLed(int index, Color color) {
        buffer.setLED(index, color);
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        this.buffer = buffer;
    }
}
