package frc.robot.swerve;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.swerve.ISwerveDriveIo;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

@SuppressWarnings("removal")
public class SwerveHw24 implements ISwerveDriveIo {

    //Swerve corner locations for kinematics
    // 22.75"x17.25" (5.5" above center)
    private Translation2d[] swervePositions = {
        new Translation2d(Units.inchesToMeters(17.25/2), Units.inchesToMeters(22.75/2)),  //convert inches to meters. y is front to back. left front is 1st wheel
        new Translation2d(Units.inchesToMeters(17.25/2), Units.inchesToMeters(-22.75/2)),  //front right wheel
        new Translation2d(Units.inchesToMeters(-17.25/2), Units.inchesToMeters(22.75/2)),  // rear left
        new Translation2d(Units.inchesToMeters(-17.25/2), Units.inchesToMeters(-22.75/2))  // rear right
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
    private double absoluteAngle[];
    
    public SwerveHw24() {

        //allocate our hardware
        int NUM_MOTORS = swervePositions.length;
        driveMotors = new TalonFX[NUM_MOTORS];
        turnMotors = new CANSparkMax[NUM_MOTORS];
        turnSensors = new CANCoder[NUM_MOTORS];
        correctedAngle = new double[NUM_MOTORS];
        absoluteAngle = new double[NUM_MOTORS];
        turnEncoder = new RelativeEncoder[NUM_MOTORS];
        turnPid = new PIDController[NUM_MOTORS];

        //FL
        driveMotors[0] = new TalonFX(11, RobotContainer.kCanBusName);  
        turnMotors[0] = new CANSparkMax(12, MotorType.kBrushless);  
        turnSensors[0] = new CANCoder(13, RobotContainer.kCanBusName);  
        
        //FR
        driveMotors[1] = new TalonFX(21, RobotContainer.kCanBusName);  
        turnMotors[1] = new CANSparkMax(22, MotorType.kBrushless);  
        turnSensors[1] = new CANCoder(23, RobotContainer.kCanBusName);  

        //RL
        driveMotors[2] = new TalonFX(31, RobotContainer.kCanBusName);  
        turnMotors[2] = new CANSparkMax(32, MotorType.kBrushless);  
        turnSensors[2] = new CANCoder(33, RobotContainer.kCanBusName);  

        //RR
        driveMotors[3] = new TalonFX(41, RobotContainer.kCanBusName);  
        turnMotors[3] = new CANSparkMax(42, MotorType.kBrushless);  
        turnSensors[3] = new CANCoder(43, RobotContainer.kCanBusName);  

        for (CANCoder sensor: turnSensors) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 100);
        }
        configureMotors();

        //register stuff for logging
        for(int wheel = 0; wheel<NUM_MOTORS; wheel++) {
            final int wheelFinal = wheel;
            Logger.RegisterCanSparkMax(moduleNames[wheel] + " Turn", turnMotors[wheel]);
            Logger.RegisterTalon(moduleNames[wheel] + " Drive", driveMotors[wheel]);
            Logger.RegisterCanCoder(moduleNames[wheel] + " Abs", turnSensors[wheel]);
            Logger.RegisterSensor(moduleNames[wheel] + " Speed", () -> getCornerSpeed(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Turn Pos", () -> getCornerAngle(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Drive Dist", () -> getCornerDistance(wheelFinal));
            Logger.RegisterSensor(moduleNames[wheel] + " Raw Rotations", () -> driveMotors[wheelFinal].getPosition().getValueAsDouble());

            //initialize hardware
            turnEncoder[wheel] = turnMotors[wheel].getEncoder();
            turnEncoder[wheel].setPositionConversionFactor(176.31/10.4752);
            turnPid[wheel] = new PIDController(.6/Math.PI, .15, 0);

            //from https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
            turnMotors[wheel].setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
            turnMotors[wheel].setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);

        }
        setDriveMotorBrakeMode(true);
        setTurnMotorBrakeMode(true);
    }

    public void configureMotors() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.Slot0.kP = 0;
        talonFXConfigs.Slot0.kI = 0;
        talonFXConfigs.Slot0.kD = 0;
        talonFXConfigs.Slot0.kS = 0.13;
        talonFXConfigs.Slot0.kG = 0;
        talonFXConfigs.Slot0.kV = 2.8;
        talonFXConfigs.Slot0.kA = 0.215;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.SupplyCurrentThreshold = 50;
        talonFXConfigs.CurrentLimits.SupplyTimeThreshold = 0.2;

        talonFXConfigs.MotorOutput.PeakForwardDutyCycle = 1;
        talonFXConfigs.MotorOutput.PeakReverseDutyCycle = -1;
        talonFXConfigs.Voltage.PeakForwardVoltage = 16;
        talonFXConfigs.Voltage.PeakReverseVoltage = 16;

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = metersToRotations(UtilFunctions.getSetting("Swerve Drive/Max Speed", 5.0));
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = metersToRotations(UtilFunctions.getSetting("Swerve Drive/Max Wheel Accel", 42.0));

        for(TalonFX motor : driveMotors) {            
            motor.getConfigurator().apply(talonFXConfigs);
            motor.setPosition(0);
            motor.setInverted(false);
        }

        for(int wheel=0; wheel<swervePositions.length; wheel++) {
            turnMotors[wheel].setInverted(true);
            turnMotors[wheel].setSmartCurrentLimit(40, 25);
            turnMotors[wheel].burnFlash();
        }
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        //this is a hack as the wheel encoder reports wrong
        if (wheel == 1) {
            return absoluteAngle[wheel] + 180;
        }
        return absoluteAngle[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnEncoder[wheel].getPosition();
    }

    @Override
    public double getCornerDistance(int wheel) {
        return rotationsToMeters(driveMotors[wheel].getPosition().getValueAsDouble());
    }

    @Override
    public Translation2d[] getCornerLocations() {
        return swervePositions;
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return rotationsToMeters(driveMotors[wheel].getVelocity().getValueAsDouble());
    }

  @Override
    public String[] getModuleNames() {
        return moduleNames;
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        //hardware test in % output mode
        if(!DriverStation.isTest()) {
            //double velPct = swerveModuleState.speedMetersPerSecond / 5;  //TODO set equal to max module speed
            //driveMotors[wheel].set(velPct);
        }
        
        //velPct = (swerveModuleState.angle.getDegrees() - correctedAngle[wheel])*.2/90;
        //turnMotors[wheel].set(velPct);


        //PID control
        if(Math.abs(swerveModuleState.speedMetersPerSecond) > 0.1) {
            driveMotors[wheel].setControl(new MotionMagicVelocityVoltage(swerveModuleState.speedMetersPerSecond));
        } else {
            driveMotors[wheel].set(0);
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
        NeutralModeValue mode;

        if(brakeOn) {
            mode = NeutralModeValue.Brake;
        } else {
            mode = NeutralModeValue.Coast;
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
        for (int wheel = 0; wheel < turnSensors.length; wheel++) {
            var newAngle = turnSensors[wheel].getAbsolutePosition();
            absoluteAngle[wheel] = MathUtil.inputModulus(newAngle, absoluteAngle[wheel] - 180, absoluteAngle[wheel] + 180);
        }
    }

    @Override
    public void resetWheelPositions() {
        for(int wheel = 0; wheel < driveMotors.length; wheel++) {
            driveMotors[wheel].setPosition(0);
        }
    }


    private final double kWheelRadiusInches = 2;    //4" wheels
    private final double kGearRatio = 6.75;         //L2 gearing

    private double rotationsToMeters(double rotations) {
        /* Get circumference of wheel */
        final double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the meters traveled per rotation */
        final double metersPerWheelRotation = Units.inchesToMeters(circumference);
        /* Now apply gear ratio to input rotations */
        double gearedRotations = rotations / this.kGearRatio;
        /* And multiply geared rotations by meters per rotation */
        return gearedRotations * metersPerWheelRotation;
    }

    private double metersToRotations(double meters) {
        /* Get circumference of wheel */
        final double circumference = this.kWheelRadiusInches * 2 * Math.PI;
        /* Every rotation of the wheel travels this many inches */
        /* So now get the rotations per meter traveled */
        final double wheelRotationsPerMeter = 1.0 / Units.inchesToMeters(circumference);
        /* Now apply wheel rotations to input meters */
        double wheelRotations = wheelRotationsPerMeter * meters;
        /* And multiply by gear ratio to get rotor rotations */
        return wheelRotations * this.kGearRatio;
    }

    @Override
    public double getDriveVoltage(int wheel) {
        return driveMotors[wheel].getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setDriveVoltage(int wheel, double volts) {
        driveMotors[wheel].setControl(new VoltageOut(volts));
    }
}
