package org.livoniawarriors.odometry;

public interface IGyroHardware {
    void updateHardware();

    double getGyroAngle();
    double getPitchAngle();
    double getRollAngle();

    double getXAccel();
    double getYAccel();
    double getZAccel();
}
