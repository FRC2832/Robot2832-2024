package frc.robot.hardware;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.swerve.ISwerveDriveIo;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@SuppressWarnings("removal")
public class PracticeSwerveHw implements ISwerveDriveIo {

    //measuring the robot, we got 13899 counts/rev, theoretical is 13824 counts/rev (L2 gear set at 6.75:1 ratio)
    //needs to be scaled * 39.37 (in/m) / (4"*Pi wheel diameter) / 10 (units per 100ms) = 43311
    //the scale factor is average of 4 wheels/measured distance
    private final double COUNTS_PER_METER = 43311 / 1.002855;     //velocity units
    private final double VELO_PER_METER = COUNTS_PER_METER/10;        //distance units

    //Swerve corner locations for kinematics
    //24.75" square
    private Translation2d[] swervePositions = {
        new Translation2d(0.314, 0.314),  //convert inches to meters. y is front to back. left front is 1st wheel
        new Translation2d(0.314, -0.314),  //front right wheel
        new Translation2d(-0.314, 0.314),  // rear left
        new Translation2d(-0.314, -0.314)  // rear right
    };

    private String[] moduleNames = {
        "FL",
        "FR",
        "RL",
        "RR"
    };

    private TalonFX driveMotors[];  //TODO: upgrade to Phoenix 6 once this works
    private CANSparkMax turnMotors[];
    private RelativeEncoder turnEncoder[];
    private CANCoder turnSensors[];
    private PIDController turnPid[];

    private double correctedAngle[];
    
    public PracticeSwerveHw() {

        //allocate our hardware
        int NUM_MOTORS = swervePositions.length;
        driveMotors =new TalonFX[NUM_MOTORS];
        turnMotors = new CANSparkMax[NUM_MOTORS];
        turnSensors = new CANCoder[NUM_MOTORS];
        correctedAngle = new double[NUM_MOTORS];
        turnEncoder = new RelativeEncoder[NUM_MOTORS];
        turnPid = new PIDController[NUM_MOTORS];

        //FL
        driveMotors[0] = new TalonFX(11);  
        turnMotors[0] = new CANSparkMax(12, MotorType.kBrushless);  
        turnSensors[0] = new CANCoder(13);  
        
        //FR
        driveMotors[1] = new TalonFX(21);  
        turnMotors[1] = new CANSparkMax(22, MotorType.kBrushless);  
        turnSensors[1] = new CANCoder(23);  

        //RL
        driveMotors[2] = new TalonFX(31);  
        turnMotors[2] = new CANSparkMax(32, MotorType.kBrushless);  
        turnSensors[2] = new CANCoder(33);  

        //RR
        driveMotors[3] = new TalonFX(41);  
        turnMotors[3] = new CANSparkMax(42, MotorType.kBrushless);  
        turnSensors[3] = new CANCoder(43);  

        for (CANCoder sensor: turnSensors) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);
        }

        double maxSpeed = UtilFunctions.getSetting("Swerve Drive/Max Speed", 5.0);
        TalonFXConfiguration allConfigs = new TalonFXConfiguration();
        for(TalonFX motor : driveMotors) {
            //motors MUST be reset every powerup!!!
            motor.configFactoryDefault();
            motor.getAllConfigs(allConfigs);
            //old software pid values are p 0.5, i 0.03, d 0
            allConfigs.slot0.kP = 1023 / (0.5 * COUNTS_PER_METER);
            //old value of 0.03 means if we had 1m/s error for 1 second, add 0.03V to the PID (or 1.705 counts of 1023)
            //unknown why we need 2/3, but then the math works...
            allConfigs.slot0.kI = 0.01 * 1023 / COUNTS_PER_METER * (2./3);
            //allConfigs.slot0.kI = 0;
            allConfigs.slot0.kD = 0;
            //this works out to 1023 / Max speed in counts
            allConfigs.slot0.kF = 1023 / (maxSpeed * VELO_PER_METER);

            allConfigs.slot0.integralZone = 0;
            allConfigs.slot0.allowableClosedloopError = 0;

            //the maximum velocity we want the motor to go
            allConfigs.motionCruiseVelocity = maxSpeed * VELO_PER_METER;
            //the maximum acceleration we want the motor to go
            allConfigs.motionAcceleration = 5 * VELO_PER_METER;

            allConfigs.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
            motor.configAllSettings(allConfigs);
            motor.setSelectedSensorPosition(0);

            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 40);
        }

        //register stuff for logging
        for(int wheel = 0; wheel<NUM_MOTORS; wheel++) {
            final int wheelFinal = wheel;
            Logger.RegisterCanSparkMax(moduleNames[wheel] + " Turn", turnMotors[wheel]);
            Logger.RegisterTalon(moduleNames[wheel] + " Drive", driveMotors[wheel]);
            Logger.RegisterCanCoder(moduleNames[wheel] + " Abs", turnSensors[wheel]);
            Logger.RegisterSensor(moduleNames[wheel] + " Speed", () -> getCornerSpeed(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Turn Pos", () -> getCornerAngle(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Drive Dist", () -> getCornerDistance(wheelFinal));

            //initialize hardware
            turnEncoder[wheel] = turnMotors[wheel].getEncoder();
            turnEncoder[wheel].setPositionConversionFactor(176.31/10.4752);
            turnPid[wheel] = new PIDController(.5/Math.PI, .2, 0);
            turnMotors[wheel].setInverted(true);
        }
        setDriveMotorBrakeMode(true);
        setTurnMotorBrakeMode(true);
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return turnSensors[wheel].getAbsolutePosition();
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnEncoder[wheel].getPosition();
    }

    @Override
    public double getCornerDistance(int wheel) {
        return driveMotors[wheel].getSelectedSensorPosition() / COUNTS_PER_METER;
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return swervePositions;
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveMotors[wheel].getSelectedSensorVelocity() / VELO_PER_METER;
    }

    @Override
    public String[] getModuleNames() {
        return moduleNames;
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        //hardware test in % output mode
        //double velPct = swerveModuleState.speedMetersPerSecond / 5;  //TODO set equal to max module speed
        //driveMotors[wheel].set(TalonFXControlMode.PercentOutput, velPct);
        
        //velPct = (swerveModuleState.angle.getDegrees() - correctedAngle[wheel])*.2/90;
        //turnMotors[wheel].set(velPct);


        //PID control
        if(Math.abs(swerveModuleState.speedMetersPerSecond) > 0.1) {
            driveMotors[wheel].set(TalonFXControlMode.Velocity, swerveModuleState.speedMetersPerSecond * VELO_PER_METER);
        } else {
            driveMotors[wheel].set(TalonFXControlMode.PercentOutput, 0);
        }
        //set the turn command
        //we need the request to be within the boundaries, not wrap around the 180 point
        double turnRequest = MathUtil.inputModulus(swerveModuleState.angle.getDegrees(), correctedAngle[wheel]-180, correctedAngle[wheel]+180);
        if (Math.abs(correctedAngle[wheel] - turnRequest) < 1) {
            //reset the PID to remove all the I term error so we don't overshoot and rebound
            turnPid[wheel].reset();
        }
        double turnOutput = turnPid[wheel].calculate(Math.toRadians(correctedAngle[wheel]), Math.toRadians(turnRequest));
        turnMotors[wheel].set(turnOutput);
    }

    @Override
    public void setCorrectedAngle(int wheel, double angle) {
        correctedAngle[wheel] = angle; 
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }
        
        for (TalonFX motor : driveMotors) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        IdleMode mode;

        if(brakeOn) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }
        
        for (CANSparkMax motor : turnMotors) {
            motor.setIdleMode(mode);
        }
    }

    @Override
    public void updateInputs() {
        
    }

    @Override
    public void resetWheelPositions() {
        for(int wheel = 0; wheel < driveMotors.length; wheel++) {
            driveMotors[wheel].setSelectedSensorPosition(0);
        }
    }
}
