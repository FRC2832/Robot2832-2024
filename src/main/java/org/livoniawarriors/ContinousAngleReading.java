package org.livoniawarriors;

/** This class helps handle rollover when an absolute sensor reading is not continous
 * Like most sensors only read -180 to 179 degrees and rolls back to -180.
 */
public class ContinousAngleReading {
    double angle;
    double oldReading;
    int rotations;

    public ContinousAngleReading() {
        angle = 0;
        oldReading = 0;
        rotations = 0;
    }

    /**
     * 
     * @param reading
     * @return Value in degrees
     */
    public double update(double reading) {
        //check to see if we go over a rotation and compensate with the absolute sensor
        var delta = reading - oldReading;
        if(Math.abs(delta) > 300) {
            rotations -= Math.signum(delta);
        }
        angle = reading + (rotations * 360);
        oldReading = reading;
        return angle;
    }

    /**
     * 
     * @return Value in degrees
     */
    public double getAngle() {
        return angle;
    }
}
