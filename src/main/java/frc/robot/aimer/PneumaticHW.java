package frc.robot.aimer;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

@SuppressWarnings("removal")
public class PneumaticHW implements IPneumaticHW {
    private Pigeon2 angSensor;
    private Solenoid upSolenoid, downSolenoid;
    private double currentAngle;
    BooleanLogEntry driveUp;
    BooleanLogEntry driveDown;
    DoubleLogEntry angleLog;
    boolean use1msLogging = false;

    public PneumaticHW() {
        this.angSensor = new Pigeon2(10);
        upSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 15);
        downSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

        Logger.RegisterSensor("Shooter Angle", ()->currentAngle);

        //turn on 1ms logging
        if(use1msLogging) {
            angSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,5);
            var log = DataLogManager.getLog();
            var timestamp = WPIUtilJNI.now();
            driveUp = new BooleanLogEntry(log, "Aimer:DriveUp", timestamp);
            driveDown = new BooleanLogEntry(log, "Aimer:DriveDown", timestamp);
            angleLog = new DoubleLogEntry(log, "Aimer:Angle", timestamp);
            UtilFunctions.addPeriodic(this::run1ms, 0.005, 0);
        } else {
            angSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR,18);
        }
    }

    public void run1ms() {
        //log data
        currentAngle = angSensor.getRoll();
        angleLog.append(currentAngle);
    }

    @Override
    public void updateInputs() {
        currentAngle = angSensor.getRoll();
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
}
