package frc.robot.subsystems;

import org.livoniawarriors.Logger;
import org.livoniawarriors.swerve.ISwerveDriveIo;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;

@SuppressWarnings("removal")
public class PracticeSwerveHw implements ISwerveDriveIo {

    //measuring the robot, we got 13899 counts/rev, theoretical is 13824 counts/rev (L2 gear set at 6.75:1 ratio)
    //needs to be scaled * 39.37 (in/m) / (4"*Pi wheel diameter) / 10 (units per 100ms)
    private final double COUNTS_PER_METER = 4331.1 / 0.94362;     //velocity units
    private final double VELO_PER_METER = COUNTS_PER_METER*10;        //distance units

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
        driveMotors[0] = new TalonFX(11);  // TODO: update with correct info when receive it
        turnMotors[0] = new CANSparkMax(12, MotorType.kBrushless);  // TODO: update with correct info when receive it
        turnSensors[0] = new CANCoder(13);  // TODO: update with correct info when receive it
        
        //FR
        driveMotors[1] = new TalonFX(21);  // TODO: update with correct info when receive it
        turnMotors[1] = new CANSparkMax(22, MotorType.kBrushless);  // TODO: update with correct info when receive it
        turnSensors[1] = new CANCoder(23);  // TODO: update with correct info when receive it

        //RL
        driveMotors[2] = new TalonFX(31);  // TODO: update with correct info when receive it
        turnMotors[2] = new CANSparkMax(32, MotorType.kBrushless);  // TODO: update with correct info when receive it
        turnSensors[2] = new CANCoder(33);  // TODO: update with correct info when receive it

        //RR
        driveMotors[3] = new TalonFX(41);  // TODO: update with correct info when receive it
        turnMotors[3] = new CANSparkMax(42, MotorType.kBrushless);  // TODO: update with correct info when receive it
        turnSensors[3] = new CANCoder(43);  // TODO: update with correct info when receive it

        for (CANCoder sensor: turnSensors) {
            sensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 18);
            sensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);
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
            turnPid[wheel] = new PIDController(.5/Math.PI, .2, 0);
        }
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
        //set the drive command
        double velPct = swerveModuleState.speedMetersPerSecond / 5;  //TODO set equal to max module speed
        driveMotors[wheel].set(TalonFXControlMode.PercentOutput, velPct);

        //set the turn command
        //we need the request to be within the boundaries, not wrap around the 180 point
        double turnRequest = MathUtil.inputModulus(swerveModuleState.angle.getDegrees(), correctedAngle[wheel]-180, correctedAngle[wheel]+180);
        if (Math.abs(correctedAngle[wheel] - turnRequest) < 1) {
            //reset the PID to remove all the I term error so we don't overshoot and rebound
            turnPid[wheel].reset();
        }
        double turnOutput = -turnPid[wheel].calculate(Math.toRadians(correctedAngle[wheel]), Math.toRadians(turnRequest));
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
    
}
