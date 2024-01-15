package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ILedSubsystem extends Subsystem {
    /**
     * Return the length of the LED strip
     * @return The length of the LED strip
     */
    int getLength();

    /**
     * Set the full LED buffer to show a pattern
     * @param buffer 
     */
    void setData(AddressableLEDBuffer buffer);
    
    /**
     * Return the color of the LED at the index
     * @param index What LED number to get the color of
     * @return The color of the LED at the index
     */
    Color getLed(int index);
    
    /**
     * Set the LED color at the specified position
     * @param index What LED to change the color
     * @param color What color to set it to
     */
    void setLed(int index, Color color);
}
