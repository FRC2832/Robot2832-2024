package frc.robot.aimer;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.function.DoubleSupplier;

public abstract class Aimer extends SubsystemBase {
    public abstract void updateInputs();
    public abstract double getAngle();
    public abstract void driveUp();
    public abstract void driveDown();
    public abstract void stop();
    public abstract void startPulse(double time, boolean goingUp);

    private double moe = 4.00;
    private double timeToStartUp = 0.07;
    private double timeToStartDown = 0.0825;
    private double timeToStopUp = 0.161;
    private double timeToStopDown = 0.15;
    private BufferedReader reader = null;
    private InterpolatingDoubleTreeMap upTable;
    private InterpolatingDoubleTreeMap downTable;
    private double stopAimTime = 0;

    public Aimer() {
        super();
        upTable = new InterpolatingDoubleTreeMap();
        downTable = new InterpolatingDoubleTreeMap();
        readShooterTable("/Aimer_Cal2_Up.csv",upTable);
        readShooterTable("/Aimer_Cal2_Down.csv",downTable);
    }

    public void goTo(double target) {
        double currentAngle = getAngle();
        if(((target - moe) < currentAngle) && (currentAngle < (target + moe))){
            stop(); 
        }
        else {
            if(currentAngle < target) {
                driveUp(); 
            } else {
                driveDown();
            }
        }
    }

    public void goToSmooth(double target){
        //don't allow a command if one is already running
        if (Timer.getFPGATimestamp() < stopAimTime) {
            return;
        }
        double currentAngle = getAngle();
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
            startPulse(time, goingUp);
            stopAimTime = Timer.getFPGATimestamp() + time;
        }
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    public Command driveAimer(DoubleSupplier pctCommand) {
        return run(() -> {
            double command = pctCommand.getAsDouble();
            if(command > 0.2) {
                driveDown();
            } else if (command < -0.2) {
                driveUp();
            } else {
                stop();
            }
        }).finallyDo(this::stop).withName("AimerDrive");
    }

    public Command setAimer(DoubleSupplier angle) {
        return run(() -> {
            double ang = angle.getAsDouble();
            goToSmooth(ang);
        })
        .until(() -> Math.abs(getAngle() - angle.getAsDouble()) < 3)
        .finallyDo(this::stop).withName("AimerSet");
    }

    private void readShooterTable(String fileName, InterpolatingDoubleTreeMap table){
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