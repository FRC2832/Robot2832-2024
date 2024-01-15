package org.livoniawarriors.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveDriveSim implements ISwerveDriveIo {
    private double absAngle[];
    private double turnAngle[];
    private double correctedAngle[];
    private double driveSpeed[];
    private double driveDist[];
    private ControlMode driveCommand[];
    private ControlMode turnCommand[];
    private double drivePower[];
    private double turnPower[];

    private FlywheelSim turnMotorSim[];
    private PIDController turningPIDController[];

    private final double Kv_Turn = 0.006531;
    private final double kMaxSpeed = 5;     //5m/s ~= 15ft/s
    private final double kNomBatVolt = 12.5;

    private Translation2d[] swervePositions = {
        new Translation2d(0.291, 0.291),
        new Translation2d(0.291, -0.291),
        new Translation2d(-0.291, 0.291),
        new Translation2d(-0.291, -0.291),
    };

    private String[] moduleNames = {
        "FL",
        "FR",
        "RL",
        "RR"
    };

    public SwerveDriveSim() {
        int numWheels = swervePositions.length;
        absAngle = new double[numWheels];
        turnAngle = new double[numWheels];
        correctedAngle = new double[numWheels];
        driveSpeed = new double[numWheels];
        driveDist = new double[numWheels];

        driveCommand = new ControlMode[numWheels];
        turnCommand = new ControlMode[numWheels];
        drivePower = new double[numWheels];
        turnPower = new double[numWheels];

        turnMotorSim = new FlywheelSim[numWheels];
        turningPIDController = new PIDController[numWheels];
        for(int i=0; i<numWheels; i++) {
            //kv = Volt Seconds per Meter
            //ka = ka VoltSecondsSquaredPerMotor
            turnMotorSim[i] = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.3850, 0.0385),
                DCMotor.getFalcon500(1), 150f/7);
            
            //scale factor for hardware PID to software PID
            //360/2048 is 360 degrees per rev/encoder counts per rev divided by gear ratio
            var k = (360f/2048) * (7f/150);
            turningPIDController[i] = new PIDController(1.5 * k, 0.0005 * k, 0 * k, 0.001);
        }
    }
    
    @Override
    public void updateInputs() {
        //TODO: Simulate the actual swerve corners... https://www.chiefdelphi.com/t/sysid-gains-on-sds-mk4i-modules/400373/7
        for(int i=0; i<driveCommand.length; i++) {
            //process drive command
            if (driveCommand[i] == ControlMode.MotionMagic) {
                //drive power is actually distance traveled
                driveSpeed[i] = drivePower[i] / TimedRobot.kDefaultPeriod;
            } else if (driveCommand[i] == ControlMode.Velocity) {
                driveSpeed[i] = drivePower[i];
            } else if (driveCommand[i] == ControlMode.PercentOutput) {
                driveSpeed[i] = drivePower[i] * kMaxSpeed;
            } else {
                driveSpeed[i] = 0;
            }
            driveDist[i] += driveSpeed[i] * TimedRobot.kDefaultPeriod;

            //reset drive command back to zero
            driveCommand[i] = ControlMode.Disabled;
            drivePower[i] = 0;

            //process turn command
            if(turnCommand[i] == ControlMode.Position) {
                for(var loops = 0; loops < TimedRobot.kDefaultPeriod / 0.001; loops++) {
                    double turnOutput = turningPIDController[i].calculate(correctedAngle[i], turnPower[i]);
                    //update the sensor values
                    turnAngle[i] += turnOutput;
                    absAngle[i] += turnOutput;
                }
            } else if (turnCommand[i] == ControlMode.PercentOutput) {
                var turnOutput = -turnPower[i] * kNomBatVolt * TimedRobot.kDefaultPeriod / Kv_Turn;
                //update the sensor values
                turnAngle[i] += turnOutput;
                absAngle[i] += turnOutput;
                turnPower[i] = 0;
            } else {
            }

            //reset turn command back to zero
            turnCommand[i] = ControlMode.Disabled;
            turnPower[i] = 0;
        }
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absAngle[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveSpeed[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        driveCommand[wheel] = ControlMode.Velocity;
        drivePower[wheel] = swerveModuleState.speedMetersPerSecond;
        turnCommand[wheel] = ControlMode.Position;
        //we need the request to be within the boundaries, not wrap around the 180 point
        turnPower[wheel] = MathUtil.inputModulus(swerveModuleState.angle.getDegrees(), correctedAngle[wheel]-180, correctedAngle[wheel]+180);
    }

    @Override
    public double getCornerDistance(int wheel) {
        return driveDist[wheel];
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
