package org.livoniawarriors.swerve;

import org.livoniawarriors.ContinousAngleReading;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.odometry.Odometry;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveDriveTrain extends SubsystemBase {
    /** The fastest rate we want the drive wheels to change speeds in m/s */
    final String MAX_ACCEL_KEY = "Swerve Drive/Max Wheel Accel";
    /** The fastest rate we want the swerve wheels to turn in deg/s */
    final String MAX_OMEGA_KEY = "Swerve Drive/Max Wheel Omega";
    /** The max speed possible with the swerve wheels in m/s */
    final String MIN_SPEED_KEY = "Swerve Drive/Min Speed";
    final String MAX_SPEED_KEY = "Swerve Drive/Max Speed";
    /** The angle in degrees we want the swerve to invert the request to get to position faster */
    final String OPTIMIZE_ANGLE_KEY = "Swerve Drive/Optimize Angle";
    final String FIELD_ORIENTED = "Swerve Drive/Field Oriented";

    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModulePosition[] swervePositions;
    private SwerveModuleState[] swerveTargets;
    private double gyroOffset = 0;
    private PIDController pidZero = new PIDController(0.08, 0, 0);
    private SwerveModuleState[] swerveStates;
    private boolean optimize;
    private boolean resetZeroPid;
    private double minSpeed;
    private double maxSpeed;
    private Rotation2d currentHeading;
    private Rotation2d fieldOffset;
    private Odometry odometry;
    private boolean lastTeleop;
    private ContinousAngleReading requestContinous[];

    //input settings
    private DoubleSubscriber driverMaxSpeed;
    private DoubleSubscriber driverMaxOmega;
    private DoubleSubscriber[] wheelOffsetSetting;
    
    //output data
    private DoublePublisher[] wheelCalcAngle;
    private DoublePublisher[] wheelCommandAngle;
    private DoublePublisher[] wheelRequestAngle;
    private DoublePublisher[] wheelCommandSpeed;
    private DoublePublisher[] wheelRequestSpeed;
    private DoublePublisher swerveXSpeed;
    private DoublePublisher swerveYSpeed;
    private DoublePublisher swerveOmega;
    private DoublePublisher swerveCurrentHeading;
    private DoublePublisher swerveGyroOffset;
    private DoublePublisher swerveFieldOffset;
    private DoublePublisher pidZeroError;
    private BooleanEntry aimedAtTarget;
    private DoubleArrayPublisher swerveStatePub;
    private DoubleArrayPublisher swerveRequestPub;
    private double targetAngleRad;

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    // Create a new SysId routine for characterizing the drive.
    private final SysIdRoutine m_sysIdRoutine;

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo, Odometry odometry) {
        super();
        this.hardware = hSwerveDriveIo;
        this.odometry = odometry;
        optimize = true;
        resetZeroPid = false;
        int numWheels = hardware.getCornerLocations().length;
        fieldOffset = new Rotation2d();
        currentHeading = new Rotation2d();

        //initialize module names
        String[] moduleNames = hardware.getModuleNames();

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(hSwerveDriveIo.getCornerLocations());
        
        //initialize the swerve states
        swervePositions = new SwerveModulePosition[numWheels];
        swerveTargets = new SwerveModuleState[numWheels];
        swerveStates = new SwerveModuleState[numWheels];
        requestContinous = new ContinousAngleReading[numWheels];
        wheelOffsetSetting = new DoubleSubscriber[numWheels];
        wheelCalcAngle = new DoublePublisher[numWheels];
        wheelCommandAngle = new DoublePublisher[numWheels];
        wheelRequestAngle = new DoublePublisher[numWheels];
        wheelCommandSpeed = new DoublePublisher[numWheels];
        wheelRequestSpeed = new DoublePublisher[numWheels];
        for(int wheel = 0; wheel < numWheels; wheel++) {
            swervePositions[wheel] = new SwerveModulePosition();
            swerveTargets[wheel] = new SwerveModuleState();
            swerveStates[wheel] = new SwerveModuleState();
            requestContinous[wheel] = new ContinousAngleReading();
            wheelOffsetSetting[wheel] = UtilFunctions.getSettingSub("/Swerve Drive/Wheel Offset " + moduleNames[wheel] + " (deg)", 0);
            wheelCalcAngle[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Calc Angle (deg)", 0.);
            wheelCommandAngle[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Command Angle (deg)", 0.);
            wheelRequestAngle[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Request Angle (deg)", 0.);
            wheelCommandSpeed[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Command Speed (mps)", 0.);
            wheelRequestSpeed[wheel] = UtilFunctions.getNtPub("/Swerve Drive/Module " + moduleNames[wheel] + "/Request Speed (mps)", 0.);
        }

        /** How fast we want the driver to go during normal operation in m/s */
        driverMaxSpeed = UtilFunctions.getSettingSub("/Swerve Drive/Max Driver Speed (mps)", 3);
        /** How fast we want the driver to turn during normal operation in deg/s */
        driverMaxOmega = UtilFunctions.getSettingSub("/Swerve Drive/Max Driver Omega (dps)", 625);   //1.8 * Pi rad/sec
        
        swerveXSpeed = UtilFunctions.getNtPub("/Swerve Drive/X Speed (mps)", 0.);
        swerveYSpeed = UtilFunctions.getNtPub("/Swerve Drive/Y Speed (mps)", 0.);
        swerveOmega = UtilFunctions.getNtPub("/Swerve Drive/Omega (dps)", 0.);
        swerveCurrentHeading = UtilFunctions.getNtPub("/Swerve Drive/Current Heading", 0.);
        swerveGyroOffset = UtilFunctions.getNtPub("/Swerve Drive/Gyro Offset", 0.);
        swerveFieldOffset = UtilFunctions.getNtPub("/Swerve Drive/Field Offset", 0.);
        swerveStatePub = UtilFunctions.getNtPub("/Swerve Drive/Module States", new double[0]);
        swerveRequestPub = UtilFunctions.getNtPub("/Swerve Drive/Module Requests", new double[0]);
        pidZeroError = UtilFunctions.getNtPub("/Swerve Drive/Pid Zero Error", 0.);
        aimedAtTarget = UtilFunctions.getNtEntry("/Swerve Drive/Aimed at Target", false);
        minSpeed = UtilFunctions.getSetting(MIN_SPEED_KEY, 0.5);
        maxSpeed = UtilFunctions.getSetting(MAX_SPEED_KEY, 5);


        m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Measure<Voltage> volts) -> {
                    SwerveModuleState newState = new SwerveModuleState(0, new Rotation2d());
                    for(int wheel=0; wheel<hardware.getModuleNames().length; wheel++) {
                        hardware.setDriveVoltage(wheel, volts.in(Volts));
                        hardware.setCornerState(wheel, newState);
                    }
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    log.motor("drive-fl")
                        .voltage(m_appliedVoltage.mut_replace(hardware.getDriveVoltage(0), Volts))
                        .linearPosition(m_distance.mut_replace(hardware.getCornerDistance(0), Meters))
                        .linearVelocity(m_velocity.mut_replace(hardware.getCornerSpeed(0), MetersPerSecond));
                    log.motor("drive-fr")
                        .voltage(m_appliedVoltage.mut_replace(hardware.getDriveVoltage(1), Volts))
                        .linearPosition(m_distance.mut_replace(hardware.getCornerDistance(1), Meters))
                        .linearVelocity(m_velocity.mut_replace(hardware.getCornerSpeed(1), MetersPerSecond));
                    log.motor("drive-rl")
                        .voltage(m_appliedVoltage.mut_replace(hardware.getDriveVoltage(2), Volts))
                        .linearPosition(m_distance.mut_replace(hardware.getCornerDistance(2), Meters))
                        .linearVelocity(m_velocity.mut_replace(hardware.getCornerSpeed(2), MetersPerSecond));
                    log.motor("drive-rr")
                        .voltage(m_appliedVoltage.mut_replace(hardware.getDriveVoltage(3), Volts))
                        .linearPosition(m_distance.mut_replace(hardware.getCornerDistance(3), Meters))
                        .linearVelocity(m_velocity.mut_replace(hardware.getCornerSpeed(3), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this));
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();
        currentHeading = odometry.getGyroRotation();

        //read the swerve corner state
        for(int wheel = 0; wheel < swervePositions.length; wheel++) {
            double offset = wheelOffsetSetting[wheel].get(0);
            double angle = hardware.getCornerAbsAngle(wheel) - offset;
            swervePositions[wheel].angle = Rotation2d.fromDegrees(angle);
            swervePositions[wheel].distanceMeters = hardware.getCornerDistance(wheel);

            swerveStates[wheel].angle = swervePositions[wheel].angle;
            swerveStates[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);

            wheelCalcAngle[wheel].set(angle);
            hardware.setCorrectedAngle(wheel, angle);
        }

        //when we are disabled, reset the turn pids as we don't want to act on the "error" when reenabled
        boolean curTeleop = DriverStation.isTeleopEnabled();
        if(lastTeleop == false && curTeleop == true || resetZeroPid) {
            gyroOffset = currentHeading.getDegrees();
            pidZero.reset();
        }
        lastTeleop = curTeleop;
        resetZeroPid = false;

        PushSwerveStates(swerveStates,swerveTargets);
        minSpeed = UtilFunctions.getSetting(MIN_SPEED_KEY, 0.5);
        maxSpeed = UtilFunctions.getSetting(MAX_SPEED_KEY, 5);
        swerveCurrentHeading.set(currentHeading.getDegrees());
        swerveGyroOffset.set(gyroOffset);
        swerveFieldOffset.set(fieldOffset.getDegrees());
        pidZeroError.set(pidZero.getPositionError());

        double curAngle = odometry.getPose().getRotation().getRadians();
        double aimError = Math.abs(curAngle - targetAngleRad);
        aimError = MathUtil.inputModulus(aimError, -Math.PI, Math.PI);
        aimedAtTarget.set(Math.abs(aimError) < 0.16);
        //force the target angle to be WAY off so we don't say aim is correct next loop
        targetAngleRad = curAngle + Math.PI;
    }

    /**
     * Used by PathPlanner to set a robot command
     * @param speeds Speeds to use
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    public void SwerveDrive(double xSpeed, double ySpeed, double turn) {
        SwerveDrive(xSpeed, ySpeed, turn, UtilFunctions.getSetting(FIELD_ORIENTED, true));
    }

    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        //ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;

        if (Math.abs(turn) > 0.1) {
            //if a turn is requested, reset the zero for the drivetrain
            gyroOffset = currentHeading.getDegrees();
            pidZero.reset();
        } else {
            //straighten the robot
            double normAngle = MathUtil.inputModulus(currentHeading.getDegrees(), gyroOffset-180, gyroOffset+180);
            turn = pidZero.calculate(normAngle,gyroOffset);
        }

        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, currentHeading.minus(fieldOffset));
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }

        //calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        //sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, maxSpeed);

        //log the request
        swerveXSpeed.set(xSpeed);
        swerveYSpeed.set(ySpeed);
        swerveOmega.set(Math.toDegrees(turn));
        for(int i=0; i<requestStates.length; i++) {
            requestContinous[i].update(requestStates[i].angle.getDegrees());
            wheelRequestAngle[i].set(requestStates[i].angle.getDegrees());
            wheelRequestSpeed[i].set(requestStates[i].speedMetersPerSecond);
        }

        //filter the swerve wheels
        if(optimize) {
            requestStates = optimizeSwerve(requestStates, swerveStates, true);
        }
        setCornerStates(requestStates);
    }

    public void setWheelCommand(SwerveModuleState[] requestStates) {
        resetZeroPid = true;

        //log the request
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(requestStates);
        swerveXSpeed.set(speeds.vxMetersPerSecond);
        swerveYSpeed.set(speeds.vyMetersPerSecond);
        swerveOmega.set(Math.toDegrees(speeds.omegaRadiansPerSecond));
        for(int i=0; i<requestStates.length; i++) {
            requestContinous[i].update(requestStates[i].angle.getDegrees());
            wheelRequestAngle[i].set(requestStates[i].angle.getDegrees());
            wheelRequestSpeed[i].set(requestStates[i].speedMetersPerSecond);
        }

        //command the hardware
        if(optimize) {
            requestStates = optimizeSwerve(requestStates, swerveStates, false);
        }
        setCornerStates(requestStates);
    }

    public SwerveModuleState[] optimizeSwerve(SwerveModuleState[] requestStates, SwerveModuleState[] currentState, boolean stopTurnAtZero) {
        SwerveModuleState[] outputStates = new SwerveModuleState[requestStates.length];
        //we use a little larger optimize angle since drivers turning 90* is a pretty common operation
        double optimizeAngle = UtilFunctions.getSetting(OPTIMIZE_ANGLE_KEY, 120);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            outputStates[i] = new SwerveModuleState();

            //figure out if we should invert the request
            double angleReq = requestStates[i].angle.getDegrees();
            double curAngle = currentState[i].angle.getDegrees();
            double speedReq = requestStates[i].speedMetersPerSecond;
            double requestMod = MathUtil.inputModulus(requestContinous[i].getAngle(), curAngle - 180, curAngle + 180);
            double deltaMod = requestMod - curAngle;
            if(Math.abs(deltaMod) > optimizeAngle) {
                angleReq = angleReq - 180;
                speedReq = -requestStates[i].speedMetersPerSecond;
            }

            //check to see if the robot request is moving
            if (stopTurnAtZero && Math.abs(speedReq) < minSpeed) {
                //stop the requests if there is no movement
                outputStates[i].angle = currentState[i].angle;
                //take out minimal speed so that the motors don't jitter
                outputStates[i].speedMetersPerSecond = 0;
            } else {
                //copy the request over
                outputStates[i].angle = Rotation2d.fromDegrees(angleReq);
                outputStates[i].speedMetersPerSecond = speedReq;
            }
        }
        return outputStates;
    }

    private void PushSwerveStates(SwerveModuleState[] state, SwerveModuleState[] request) {
        int size = state.length;
        double[] states = new double[size * 2];
        double[] requests = new double[size * 2];
        for(int i=0; i<size; i++) {
            states[(i * 2)] = state[i].angle.getDegrees();
            states[(i * 2)+1] = state[i].speedMetersPerSecond;
            requests[(i * 2)] = request[i].angle.getDegrees();
            requests[(i * 2)+1] = request[i].speedMetersPerSecond;
        }
        swerveStatePub.set(states);
        swerveRequestPub.set(requests);
    }

    public void stopWheels() {
        SwerveDrive(0,0,0);
    }

    public void resetFieldOriented() {
        //make field offset 0, as odometry works out the angle with tags
        var newHeading = new Rotation2d();
        if(UtilFunctions.getAlliance() == Alliance.Red) {
            newHeading = newHeading.plus(Rotation2d.fromDegrees(180));
        }

        //tell odometry we are straight again
        Pose2d newPose = new Pose2d(odometry.getPose().getTranslation(), newHeading);
        odometry.resetPose(newPose);
        fieldOffset = currentHeading;

        //reset the zeroing pid on reset
        pidZero.reset();
        gyroOffset = fieldOffset.getDegrees();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
   
    public SwerveModulePosition[] getSwervePositions() {
        return swervePositions;
    }

    public SwerveModuleState[] getSwerveStates() {
        return swerveStates;
    }

    public void setTurnMotorBrakeMode(boolean brakeOn) {
        hardware.setTurnMotorBrakeMode(brakeOn);
    }

    public void setDriveMotorBrakeMode(boolean brakeOn) {
        hardware.setDriveMotorBrakeMode(brakeOn);
    }

    public Translation2d[] getCornerLocations() {
        return hardware.getCornerLocations();
    }

    public void setOptimizeOn(boolean enabled) {
        optimize = enabled;
    }
    
    public boolean getOptimizeOn() {
        return optimize;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public double getMaxDriverSpeed() {
        return driverMaxSpeed.get();
    }

    public double getMaxDriverOmega() {
        return Math.toRadians(driverMaxOmega.get());
    }

    public double getMinSpeed() {
        return minSpeed;
    }

    private void setCornerStates(SwerveModuleState[] states) {
        swerveTargets = states;
        for(int wheel = 0; wheel<states.length; wheel++) {
            hardware.setCornerState(wheel, states[wheel]);

            //log the commands
            wheelCommandAngle[wheel].set(states[wheel].angle.getDegrees());
            wheelCommandSpeed[wheel].set(states[wheel].speedMetersPerSecond);
        }
    }


    public void resetSwervePositions(){
        hardware.resetWheelPositions();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(swerveStates);
    }

    /**
     * Drive base radius in meters. Distance from robot center to furthest module.
     * @return Distance in meters.
     */
    public double getDriveBaseRadius() {
        double dist = 0;

        for(Translation2d module : hardware.getCornerLocations()) {
            double newDist = Math.sqrt((module.getX() * module.getX()) + (module.getY() * module.getY()));
            dist = Math.max(newDist, dist);
        }

        return dist;
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command aimAtTargetBackwards(Supplier<Pose2d> target) {
        try (var pid = new PIDController(.35/Math.PI, .45, 0)) {
            //reset the pid in case the error is too high to stop the I from going crazy
            pid.setIZone(0.4);  //in radians
            return run(() -> { 
                Pose2d robotPose = odometry.getPose();
                double angleToTargetRad = UtilFunctions.getAngle(target.get(), robotPose);

                if(UtilFunctions.getAlliance() == Alliance.Blue){
                    targetAngleRad = angleToTargetRad;
                } else {
                    targetAngleRad = Math.PI + angleToTargetRad;
                }

                //put robot angle in reference to target angle
                double robotAngleRad = robotPose.getRotation().getRadians();
                robotAngleRad = MathUtil.inputModulus(robotAngleRad, targetAngleRad - Math.PI, targetAngleRad + Math.PI);

                double angleCommand = pid.calculate(robotAngleRad, targetAngleRad);
                angleCommand = Math.toDegrees(angleCommand);
                SwerveDrive(0,0,angleCommand);
            })
            .finallyDo(()->SwerveDrive(0, 0, 0));
        }
    }

    public BooleanSupplier aimedAtTarget() {
        return aimedAtTarget::get;
    }
}
