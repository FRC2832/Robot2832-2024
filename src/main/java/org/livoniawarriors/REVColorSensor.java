package org.livoniawarriors;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This class wraps the REV ColorSensorV3 class, as the color sensor uses I2C reads that sometime lock the Rio.
 * See sampleUsage() to see how to use the sensor to match to a color.  There is no periodic function with this
 * class as the reads are done on a background thread.
 */
public class REVColorSensor {
    private ColorSensorV3 colorSensor;
    private Color color;
    private double proximity;

    /**
     * Constructs a color sensor reader.
     * @param i2cPort Port the sensor is connected to
     */
    public REVColorSensor(Port i2cPort) {
        colorSensor = new ColorSensorV3(i2cPort);
        color = Color.kBlack;

        //move color sensor read to separate thread since it sometimes locks up reading I2C
        ReadColorSensorThread thread = new ReadColorSensorThread();
        thread.start();
    }

    /**
     * Get the currently seen color. Works best when within 2 inches and perpendicular to surface of interest.
     * @return Color seen
     */
    public Color getColor() {
        return color;
    }

    /**
     * Gets how close the object is.  
     * @return 1 is close, 0 is not seen
     */
    public double getProximity() {
        return proximity;
    }

    protected void sampleUsage() {
        //init, change the color targets to what you need, or add your own
        final Color kBlueTarget = new Color(0.171, 0.421, 0.406);
        final Color kRedTarget = new Color(0.499, 0.362, 0.138);
        //a unknown target is needed to give the color matcher a place to go when empty, 
        //otherwise it will try to go to one of your targets.  Should be the idle color.
        final Color kUnknownTarget = new Color(0.269, 0.481, 0.249);
        ColorMatch m_colorMatcher = new ColorMatch();
        //add all the targets to see
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kUnknownTarget);
        //this needs to be tuned based on lighting conditions
        m_colorMatcher.setConfidenceThreshold(0.95);

        //periodic
        ColorMatchResult match = m_colorMatcher.matchClosestColor(getColor());
        if (match.color == kBlueTarget) {
            SmartDashboard.putString("Color Sensor Match", "Blue");
        } else if (match.color == kRedTarget) {
            SmartDashboard.putString("Color Sensor Match", "Red");
        } else {
            SmartDashboard.putString("Color Sensor Match", "Unknown");
        }
    }
            
    private class ReadColorSensorThread extends Thread {
        //moved to a separate thread because the color sensor sometimes lags
        public void run() {
            while(true) {
                double startTime = Timer.getFPGATimestamp();
                color = colorSensor.getColor();
                proximity = colorSensor.getProximity()/2047.;
                double endTime = Timer.getFPGATimestamp();

                //report the results
                SmartDashboard.putString("Color Sensor Color", color.toHexString());
                SmartDashboard.putNumber("Color Read TimeStamp", endTime);
                SmartDashboard.putNumber("Color Read Time", endTime - startTime);

                //wait till next loop
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    // do nothing
                }
            }
        }
    }
}
