package org.livoniawarriors.odometry;

import com.ctre.phoenix.sensors.Pigeon2;

@SuppressWarnings("removal")
public class Pigeon2Gyro implements IGyroHardware {
    private Pigeon2 pigeon;
    private double[] ypr_deg;
    private double[] xyz_mps;

    public Pigeon2Gyro(int id) {
        this(id, null);
    }

    public Pigeon2Gyro(int id, String bus_name) {
        if(bus_name != null) {
            pigeon = new Pigeon2(id, bus_name);
        } else {
            pigeon = new Pigeon2(id);
        }
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
