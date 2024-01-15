package org.livoniawarriors.leds;

import org.livoniawarriors.ColorHSV;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class ColorWave extends Command {
    ILedSubsystem leds;
    AddressableLEDBuffer m_ledBuffer;
    UpdateValues hueCalc, valueCalc;
    int sat;

    public ColorWave(ILedSubsystem leds, Color color) {
        this.leds = leds;
        addRequirements(leds);
        m_ledBuffer = new AddressableLEDBuffer(leds.getLength());
        ColorHSV hsv = ColorHSV.fromColor(color);

        hueCalc = new UpdateValues(hsv.hue-10, hsv.hue-10, 80);
        sat = (int)hsv.sat;
        valueCalc = new UpdateValues(hsv.hue-27, hsv.hue+27, 110);
    }

    @Override
    public boolean runsWhenDisabled() { return true; }
    
    @Override
    public void initialize() { 
    }

    @Override
    public void execute() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            double hue = hueCalc.get(i);
            double value = valueCalc.get(i);

            // Set the value
            m_ledBuffer.setHSV(i, (int)hue, sat, (int)value);
        }
        hueCalc.Step();
        valueCalc.Step();

        leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { }

    /**
     * Run a sine wave between 2 values
     */
    private class UpdateValues {
        private double Min, Max;
        private int Count, loop;

        /**
         * Create the object
         * @param min The smallest value to output
         * @param max The biggest value to output
         * @param count How many loops it takes to go finish a wave
         */
        public UpdateValues(double min, double max, int count) {
            this.Min = Math.max(min, 0);
            this.Max = Math.min(max, 255);
            this.Count = count;
        }

        public double get(int step) {
            double diff = (Max - Min)/2;
            double mid = diff + Min;

            double value = mid + diff * Math.sin(2 * Math.PI * (loop + step) / Count);
            return value;
        }

        public void Step() {
            loop++;
            loop %= Count;
        }
    }
}
