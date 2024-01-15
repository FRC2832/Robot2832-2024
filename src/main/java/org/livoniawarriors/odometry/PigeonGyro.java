package org.livoniawarriors.odometry;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonGyro implements IGyroHardware {
    private PigeonIMU pigeon;
    private double[] ypr_deg;
    private double[] xyz_mps;

    public PigeonGyro(int id) {
        pigeon = new PigeonIMU(id);
        ypr_deg = new double[3];
        xyz_mps = new double[3];
    }

    public PigeonGyro(TalonSRX motor) {
        pigeon = new PigeonIMU(motor);
        ypr_deg = new double[3];
        xyz_mps = new double[3];
    }

    @Override
    public void updateHardware() {
        short[] temp = new short[3];
        pigeon.getYawPitchRoll(ypr_deg);
        pigeon.getBiasedAccelerometer(temp);

        for(int i=0; i<3; i++) {
            xyz_mps[i] = ((double)temp[i])/16384;
        }
    }

    @Override
    public double getGyroAngle() {
        return ypr_deg[0];
    }

    @Override
    public double getPitchAngle() {
        return ypr_deg[1];
    }

    @Override
    public double getRollAngle() {
        return ypr_deg[2];
    }

    @Override
    public double getXAccel() {
        return xyz_mps[0];
    }

    @Override
    public double getYAccel() {
        return xyz_mps[1];
    }

    @Override
    public double getZAccel() {
        return xyz_mps[2];
    }
}
