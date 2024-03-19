package frc.robot.aimer;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PneumaticHW implements IPneumaticHW {
    private Pigeon2 angSensor;
    private Solenoid upSolenoid, downSolenoid;
    private double currentAngle;
    private StatusSignal<Double> rollAngle;
    BooleanLogEntry driveUp;
    BooleanLogEntry driveDown;
    DoubleLogEntry angleLog;
    boolean use1msLogging = false;

    public PneumaticHW() {
        this.angSensor = new Pigeon2(10);
        upSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);
        downSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

        Logger.RegisterSensor("Shooter Angle", ()->currentAngle);
        rollAngle = angSensor.getRoll();
        //turn on 1ms logging
        if(use1msLogging) {
            rollAngle.setUpdateFrequency(1000);
            var log = DataLogManager.getLog();
            var timestamp = WPIUtilJNI.now();
            driveUp = new BooleanLogEntry(log, "Aimer:DriveUp", timestamp);
            driveDown = new BooleanLogEntry(log, "Aimer:DriveDown", timestamp);
            angleLog = new DoubleLogEntry(log, "Aimer:Angle", timestamp);
            UtilFunctions.addPeriodic(this::run1ms, 0.001, 0);
        } else {
            rollAngle.setUpdateFrequency(50);
        }
    }

    public void run1ms() {
        //log data
        rollAngle.refresh();
        currentAngle = rollAngle.getValueAsDouble();
        angleLog.append(currentAngle);
    }

    @Override
    public void updateInputs() {
        rollAngle.refresh();
        currentAngle = rollAngle.getValueAsDouble();
    }

    @Override 
    public double getAngle() {
        return currentAngle;
    }

    @Override
    public void driveUp() {
        upSolenoid.set(true);
        downSolenoid.set(false);
        if(driveUp != null) {
            driveUp.append(true);
        }
    }

    @Override
    public void driveDown() {
        upSolenoid.set(false);
        downSolenoid.set(true);
        if(driveDown != null) {
            driveDown.append(true);
        }
    }

    @Override
    public void stop() {
        upSolenoid.set(false);
        downSolenoid.set(false);
        if(driveDown != null) {
            driveUp.append(false);
            driveDown.append(false);
        }
    }
    
    public void startPulse(double time, boolean goingUp){
        if(goingUp){
            upSolenoid.setPulseDuration(time);
            downSolenoid.set(false);
        }
        else{
            upSolenoid.set(false);
            downSolenoid.setPulseDuration(time);
        }
    }


}
