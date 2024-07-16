package frc.robot.aimer;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;


public class Pneumatics extends SubsystemBase {
    IPneumaticHW hardware;
    private double moe = 4.00;
    private double timeToStartUp = 0.07;
    private double timeToStartDown = 0.0825;
    private double timeToStopUp = 0.26;
    private double timeToStopDown = 0.28;
    private BufferedReader reader = null;
    private InterpolatingDoubleTreeMap upTable;
    private InterpolatingDoubleTreeMap downTable;
    private double stopAimTime = 0;

    public Pneumatics(IPneumaticHW hardware) {
        super();
        this.hardware = hardware;
        upTable = new InterpolatingDoubleTreeMap();
        downTable = new InterpolatingDoubleTreeMap();
        readShooterTable("/Aimer_Cal2_Up.csv",upTable);
        readShooterTable("/Aimer_Cal2_Down.csv",downTable);
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

    public void goToSmooth(double target){
        //don't allow a command if one is already running
        if (Timer.getFPGATimestamp() < stopAimTime) {
            return;
        }
        double currentAngle = hardware.getAngle();
        boolean goingUp = target >= currentAngle;
        double time;
        if(goingUp){
            var curTime = upTable.get(currentAngle);
            var tarTime = upTable.get(target);
            time = timeToStartUp + (tarTime - curTime) - timeToStopUp;
        }
        else{
            var curTime = downTable.get(currentAngle);
            var tarTime = downTable.get(target);
            time = timeToStartDown + (tarTime - curTime) - timeToStopDown;
        }
        SmartDashboard.putNumber("Smooth Aimer Timer", time);
        if(time<0){
            time = 0;
        } else {
            hardware.startPulse(time, goingUp);
            stopAimTime = Timer.getFPGATimestamp() + time;
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