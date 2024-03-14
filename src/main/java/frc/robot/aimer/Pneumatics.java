package frc.robot.aimer;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;


public class Pneumatics extends SubsystemBase {
    IPneumaticHW hardware;
    private double moe = 4.00;
    private BufferedReader reader = null;
    private InterpolatingDoubleTreeMap upTable;

    public Pneumatics(IPneumaticHW hardware) {
        super();
        this.hardware = hardware;
        upTable = new InterpolatingDoubleTreeMap();
        readShooterTable("/Aimer_calibration_up.csv",upTable);
    }

    public void goTo(double target) {
        double currentAngle = hardware.getAngle();
        if(((target - moe) < currentAngle) && (currentAngle < (target + moe))){
            hardware.stop(); 
        }
        else {
            if(currentAngle < target) {
                hardware.driveUp(); 
            } else {
                hardware.driveDown();
            }
        }
    }

    public void driveUp() {
        hardware.driveUp();
    }

    public void driveDown() {
        hardware.driveDown();
    }

    public void stop() {
        hardware.stop();
    }

    public double getAngle() {
        return hardware.getAngle();
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
    }

    public void readShooterTable(String fileName, InterpolatingDoubleTreeMap table){
        try {
			String path = Filesystem.getDeployDirectory() + fileName;
            reader = new BufferedReader(new FileReader(path));
			String line = reader.readLine();

			while ((line = reader.readLine()) != null) {
                var parts = line.split(",");
                if(parts.length < 2){
                    continue;
                }
                double angle = Double.parseDouble(parts[1]);
                double time = Double.parseDouble(parts[0]);
                table.put(angle,time);
			}

			reader.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
}