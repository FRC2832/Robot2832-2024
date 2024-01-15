package org.livoniawarriors.odometry;

public class BlankGyro implements IGyroHardware {
    @Override
    public void updateHardware() {
    }

    @Override
    public double getGyroAngle() {
        return 0;
    }

    @Override
    public double getPitchAngle() {
        return 0;
    }

    @Override
    public double getRollAngle() {
        return 0;
    }

    @Override
    public double getXAccel() {
        return 0;
    }

    @Override
    public double getYAccel() {
        return 0;
    }

    @Override
    public double getZAccel() {
        return 0;
    }
    
}
