package org.livoniawarriors.swerve;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

@SuppressWarnings("removal")
public class SwerveHw23 implements ISwerveDriveIo {
    //measuring the robot, we got 11114 counts/90*, the theoretical amount is 10971.428/90* (150/7:1 gear ratio, 2048 counts/rev)
    private final double COUNTS_PER_DEGREE = 121.9; //using theoretical amount

    //measuring the robot, we got 13899 counts/rev, theoretical is 13824 counts/rev (L2 gear set at 6.75:1 ratio)
    //needs to be scaled * 39.37 (in/m) / (4"*Pi wheel diameter) / 10 (units per 100ms)
    private final double COUNTS_PER_METER = 4331.1 / 0.94362;     //velocity units
    private final double DIST_PER_METER = COUNTS_PER_METER*10;        //distance units

    //motors and sensors
    private TalonFX turnMotor[];
    private TalonFX driveMotor[];
    private CANCoder absSensor[];
    private PIDController turnPid[];
    
    //sensor value buffers
    private double absSensorValue[];
    private double driveWheelVelocity[];
    private double driveWheelDistance[];
    private double turnMotorAngle[];
    private double correctedAngle[];

    //Swerve corner locations for kinematics
    //22.25 -1.5" width 10.375"
    //26.75" for the distance , 13.375"
    private Translation2d[] swervePositions = {
        new Translation2d(0.264, 0.340),
        new Translation2d(0.264, -0.340),
        new Translation2d(-0.264, 0.340),
        new Translation2d(-0.264, -0.340)
    };

    private String[] moduleNames = {
        "FL",
        "FR",
        "RL",
        "RR"
    };

    public SwerveHw23() {
        //initialize array sizes
        int numWheels = swervePositions.length;

        //initialize sensor buffers
        turnMotor = new TalonFX[numWheels];
        driveMotor = new TalonFX[numWheels];
        absSensor = new CANCoder[numWheels];
        absSensorValue = new double[numWheels];
        driveWheelVelocity = new double[numWheels];
        driveWheelDistance = new double[numWheels];
        turnMotorAngle = new double[numWheels];
        correctedAngle = new double[numWheels];

        //software turn PID setup
        turnPid = new PIDController[numWheels];
        for(int i=0; i<turnPid.length; i++) {
            turnPid[i] = new PIDController(5,1.8,0);
        }
        
        //initialize each motor/sensor
        driveMotor[0] = new TalonFX(11);
        driveMotor[1] = new TalonFX(21);
        driveMotor[2] = new TalonFX(31);
        driveMotor[3] = new TalonFX(41);

        turnMotor[0] = new TalonFX(12);
        turnMotor[1] = new TalonFX(22);
        turnMotor[2] = new TalonFX(32);
        turnMotor[3] = new TalonFX(42);

        absSensor[0] = new CANCoder(13);
        absSensor[1] = new CANCoder(23);
        absSensor[2] = new CANCoder(33);
        absSensor[3] = new CANCoder(43);

        TalonFXConfiguration allConfigs = new TalonFXConfiguration();

        for (CANCoder sensor: absSensor) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);
        }

        for(TalonFX motor : driveMotor) {
            //motors MUST be reset every power up!!!
            motor.configFactoryDefault();
            motor.getAllConfigs(allConfigs);
            //old software pid values are p 0.5, i 0.03, d 0
            allConfigs.slot0.kP = 1024 / (0.5 * COUNTS_PER_METER);
            //old value of 0.03 means if we had 1m/s error for 1 second, add 0.03V to the PID (or 1.705 counts of 1023)
            //unknown why we need 2/3, but then the math works...
            //allConfigs.slot0.kI = 0.03 * Constants.CTRE_P_RES / COUNTS_PER_METER * (2./3);
            allConfigs.slot0.kI = 0;
            allConfigs.slot0.kD = 0;
            //this works out to 1023 / Max speed in counts
            allConfigs.slot0.kF = 1023 / (5 * COUNTS_PER_METER);

            allConfigs.slot0.integralZone = 0;
            allConfigs.slot0.allowableClosedloopError = 0;

            //the maximum velocity we want the motor to go
            allConfigs.motionCruiseVelocity = 5 * COUNTS_PER_METER;
            //the maximum acceleration we want the motor to go
            allConfigs.motionAcceleration = 5 * COUNTS_PER_METER;

            motor.configAllSettings(allConfigs);
            motor.setSelectedSensorPosition(0);

            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }

        for(TalonFX motor : turnMotor) {
            //motors MUST be reset every power up!!!
            motor.configFactoryDefault();
            motor.getAllConfigs(allConfigs);
            allConfigs.slot1.kP = 0.2;
            allConfigs.slot1.kI = 0.0005;
            allConfigs.slot1.kD = 40;
            allConfigs.slot1.kF = 0;
            allConfigs.slot1.integralZone = 0;
            allConfigs.slot1.allowableClosedloopError = 300;
            allConfigs.motionCruiseVelocity = 20960;
            allConfigs.motionAcceleration = 40960;
            motor.configAllSettings(allConfigs);
            motor.selectProfileSlot(1, 0);

            //this stator current limit helps stop neutral brake faults
            StatorCurrentLimitConfiguration cfg = new StatorCurrentLimitConfiguration();
            cfg.enable = false;
            cfg.currentLimit = 20;
            cfg.triggerThresholdCurrent = 40;
            motor.configStatorCurrentLimit(cfg);

            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }

        //register stuff for logging
        for(int wheel = 0; wheel<numWheels; wheel++) {
            final int wheelFinal = wheel;
            Logger.RegisterTalon(moduleNames[wheel] + " Turn", turnMotor[wheel]);
            Logger.RegisterTalon(moduleNames[wheel] + " Drive", driveMotor[wheel]);
            Logger.RegisterCanCoder(moduleNames[wheel] + " Abs", absSensor[wheel]);
            Logger.RegisterSensor(moduleNames[wheel] + " Speed", () -> getCornerSpeed(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Turn Pos", () -> getCornerAngle(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Drive Dist", () -> getCornerDistance(wheelFinal));
        }
    }

    @Override
    public void updateInputs() {
        for(int i=0; i<swervePositions.length; i++) {
            absSensorValue[i] = absSensor[i].getAbsolutePosition();
            driveWheelVelocity[i] = driveMotor[i].getSelectedSensorVelocity() / COUNTS_PER_METER;
            driveWheelDistance[i] = driveMotor[i].getSelectedSensorPosition() / DIST_PER_METER;
            turnMotorAngle[i] = -turnMotor[i].getSelectedSensorPosition() / COUNTS_PER_DEGREE;
        }
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }

        for (TalonFX motor : turnMotor) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }
        
        for (TalonFX motor : driveMotor) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absSensorValue[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnMotorAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveWheelVelocity[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        driveMotor[wheel].set(ControlMode.Velocity, swerveModuleState.speedMetersPerSecond * COUNTS_PER_METER);
        
        //we need the request to be within the boundaries, not wrap around the 180 point
        double turnRequest = MathUtil.inputModulus(swerveModuleState.angle.getDegrees(), correctedAngle[wheel]-180, correctedAngle[wheel]+180);
        if (Math.abs(correctedAngle[wheel] - turnRequest) < 1) {
            //reset the PID to remove all the I term error so we don't overshoot and rebound
            turnPid[wheel].reset();
        }
        double turnVolts = -turnPid[wheel].calculate(Math.toRadians(correctedAngle[wheel]), Math.toRadians(turnRequest));
        turnMotor[wheel].set(ControlMode.PercentOutput, turnVolts / RobotController.getBatteryVoltage());
    }

    @Override
    public double getCornerDistance(int wheel) {
        return driveWheelDistance[wheel];
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return swervePositions;
    }

    @Override
    public String[] getModuleNames() {
        return moduleNames;
    }

    @Override
    public void setCorrectedAngle(int wheel, double angle) {
        correctedAngle[wheel] = angle;
    }
}
