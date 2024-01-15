package org.livoniawarriors;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTableEvent;

/**
 * This class lets us configure PIDs using our units that make sense.  The input units should be 
 * something like meters or degrees, and the output would be in volts, with max as 12V.
 */
@SuppressWarnings("removal")
public class PidConstants {
    private String key;

    /** 
     * How much voltage do we add based on amount of error from set point.
     * For example, if P=2, that means for each input unit we are off, add 2 volts to the output.
     * 
     * Units: Volts/error
     */
    public double kP;

    /** 
     * How much voltage do we add based on the sum of error.  This number should be small!!!
     * For example, if I=2, for each input unit we are off for 1 sec, add 2 volts to the output.
     * 
     * Units: Volts/sum of error
     */
    public double kI;

    /**
     * How much voltage do we add to dampen rapid changes.
     * 
     * Units: Volts/change in error
     */
    public double kD;

    /**
     * How many volts / max speed achievable.
     * 
     * Units Volts/request
     */
    public double kF;

    /**
     * How much voltage is needed to start the mechanism moving.  Setting to zero is usually fine.
     * 
     * Units: Volts
     */
    public double kS;

    /**
     * How much voltage is needed to overcome gravity.  Set to zero if not a vertical mechanism.
     *  
     * Units: Volts
     */
    public double kG;
    
    /**
     * Also called kF, how many sensor units change per volt.
     *  
     * Units: Volts / Rev/S
     */
    public double kV;

    /** 
     * How many sensor units^s squared change per volt.  Used to match the feed forward accelerations.
     * Set to zero in most applications.
     * 
     * Units: Volts / Rev/S^2
     */
    public double kA;

    /**
     * How much range you want before the I-Term enables.  For example, if kiZone=10, and the measurement
     * is 12 units off, ignore the I term.  OK to start at zero, this is used to stop integral windup when
     * the mechanism starts moving and usually adds more overshoot after reaching the target.
     * 
     * Units: Distance
     */
    public double kiZone;

    /**
     * How much range is allowable to turn off the PID.  For example, if kiError = 2, and the mechanism is 1
     * unit away, the PID would turn off.  OK to start at zero.  This has a side effect of basically always
     * stopping control the iError off, so probably good to keep off.
     * 
     * Units: Distance
     */
    public double kiError;

    /**
     * Used in motion control to say the max speed the mechanism should travel.
     * 
     * Units: Distance/Sec
     */
    public double kVelMax;

    /**
     * Used in motion control to say the max acceleration the mechanism should travel.
     * 
     * Units: Distance/Sec^2
     */
    public double kAccelMax;

    /**
     * Create a generic Constants table with no backing
     */
    public PidConstants() {
        kP = 0;
        kI = 0;
        kD = 0;
        kF = 0;
        kS = 0;
        kG = 0;
        kV = 0;
        kA = 0;
        kiZone = 0;
        kiError = 0;
        kVelMax = 0;
        kAccelMax = 0;
        key = null;
    }

    /**
     * Create a generic Constants table with persistent backing from NetworkTables
     * @param key NetworkTable location to save at
     */
    public PidConstants(String key) {
        String tempKey = key;
        if(!tempKey.startsWith("/", 0)) tempKey = "/" + tempKey;
        if(!tempKey.endsWith("/")) tempKey = tempKey + "/";
        this.key = tempKey;
        loadFromNT();
    }

    public void onChange(Consumer<NetworkTableEvent> listener) {
        UtilFunctions.onNtChange(key + "P", listener);
        UtilFunctions.onNtChange(key + "I", listener);
        UtilFunctions.onNtChange(key + "D", listener);
        UtilFunctions.onNtChange(key + "F", listener);
        UtilFunctions.onNtChange(key + "S", listener);
        UtilFunctions.onNtChange(key + "G", listener);
        UtilFunctions.onNtChange(key + "V", listener);
        UtilFunctions.onNtChange(key + "A", listener);
        UtilFunctions.onNtChange(key + "iZone", listener);
        UtilFunctions.onNtChange(key + "iError", listener);
        UtilFunctions.onNtChange(key + "MaxVel", listener);
        UtilFunctions.onNtChange(key + "MaxAccel", listener);
    }

    public void loadFromNT() {
        kP = UtilFunctions.getSetting(key + "P", 0);
        kI = UtilFunctions.getSetting(key + "I", 0);
        kD = UtilFunctions.getSetting(key + "D", 0);
        kF = UtilFunctions.getSetting(key + "F", 0);
        kS = UtilFunctions.getSetting(key + "S", 0);
        kG = UtilFunctions.getSetting(key + "G", 0);
        kV = UtilFunctions.getSetting(key + "V", 0);
        kA = UtilFunctions.getSetting(key + "A", 0);
        kiZone = UtilFunctions.getSetting(key + "iZone", 0);
        kiError = UtilFunctions.getSetting(key + "iError", 0);
        kVelMax = UtilFunctions.getSetting(key + "MaxVel", 0);
        kAccelMax = UtilFunctions.getSetting(key + "MaxAccel", 0);
    }

    /**
     * Configure a TalonFX (aka Falcon or Kraken) with this PID constants
     * @param motor Motor to configure
     */
    public void configureMotor(TalonFX motor) {
        //TODO Implement conversion from CTRE unit to sane units...  Should be 1023/12 / scaleFactor
        loadFromNT();
        TalonFXConfiguration allConfigs = new TalonFXConfiguration();
        motor.getAllConfigs(allConfigs);
        allConfigs.slot0.kP = kP ;
        allConfigs.slot0.kI = kI;
        allConfigs.slot0.kD = kD;
        allConfigs.slot0.kF = kF;

        allConfigs.slot0.integralZone = kiZone;
        allConfigs.slot0.allowableClosedloopError = kiError;

        //these use the velocity units, which are 10x more counts vs distance (units/100us)
        allConfigs.motionCruiseVelocity = kVelMax;
        allConfigs.motionAcceleration = kAccelMax;

        motor.configAllSettings(allConfigs);
    }

    /**
     * Configure a TalonSRX speed controller with this PID constants
     * @param motor Motor to configure
     */
    public void configureMotor(TalonSRX motor) {
        //TODO Implement conversion from CTRE unit to sane units...  Should be 1023/12 / scaleFactor
        loadFromNT();
        TalonSRXConfiguration allConfigs = new TalonSRXConfiguration();
        motor.getAllConfigs(allConfigs);
        allConfigs.slot0.kP = kP ;
        allConfigs.slot0.kI = kI;
        allConfigs.slot0.kD = kD;
        allConfigs.slot0.kF = kF;

        allConfigs.slot0.integralZone = kiZone;
        allConfigs.slot0.allowableClosedloopError = kiError;

        //these use the velocity units, which are 10x more counts vs distance (units/100us)
        allConfigs.motionCruiseVelocity = kVelMax;
        allConfigs.motionAcceleration = kAccelMax;

        motor.configAllSettings(allConfigs);
    }

    /**
     * Configure a CanSparkMax speed controller with this PID constants
     * @param motor Motor to configure
     * @return PID controller to command with PID
     */
    public SparkMaxPIDController configureMotor(CANSparkMax motor) {
        loadFromNT();
        SparkMaxPIDController pid = motor.getPIDController();
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setFF(kF);

        pid.setIZone(kiZone);
        pid.setSmartMotionAllowedClosedLoopError(kiError, 0);

        pid.setSmartMotionMaxVelocity(kVelMax, 0);
        pid.setSmartMotionMaxAccel(kAccelMax,0);

        return pid;
    }
}
