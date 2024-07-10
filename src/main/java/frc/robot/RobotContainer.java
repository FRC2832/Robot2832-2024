// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;
import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.odometry.Pigeon2Gyro;
import org.livoniawarriors.odometry.PigeonGyro;
import org.livoniawarriors.odometry.SimSwerveGyro;
import org.livoniawarriors.swerve.MoveWheels;
import org.livoniawarriors.swerve.SwerveDriveSim;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Controls.AutoFixedShot;
import frc.robot.Controls.AutoShotLookup;
import frc.robot.Controls.Autoshot;
import frc.robot.Controls.FlightDriveControls;
import frc.robot.Controls.IDriveControls;
import frc.robot.Controls.OperatorControls;
import frc.robot.Controls.OperatorStick;
import frc.robot.Controls.ShootFrom;
import frc.robot.Controls.ShooterCalibrate;
import frc.robot.Controls.XboxDriveControls;
import frc.robot.Controls.AutoShotLookup.TargetLocation;
import frc.robot.VisionSystem.FieldLocation;
import frc.robot.aimer.Aimer;
import frc.robot.aimer.AimerHw;
import frc.robot.aimer.AimerSim;
import frc.robot.amp.Amp;
import frc.robot.amp.AmpHw;
import frc.robot.amp.AmpSim;
import frc.robot.climber.DriveClimb;
import frc.robot.climber.HomeClimber;
import frc.robot.climber.Inclinator;
import frc.robot.climber.InclinatorHw;
import frc.robot.climber.InclinatorSim;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeHw;
import frc.robot.intake.IntakeSim;
import frc.robot.kicker.Kicker;
import frc.robot.kicker.KickerHw;
import frc.robot.kicker.KickerSim;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterHw;
import frc.robot.shooter.ShooterSim;
import frc.robot.swerve.DriveStick;
import frc.robot.swerve.PracticeSwerveHw;
import frc.robot.swerve.ResetWheelPosition;
import frc.robot.swerve.SwerveHw24;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static String kCanBusName = "rio";
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private SwerveDriveTrain swerveDrive;
    private Odometry odometry;
    private LedSubsystem leds;
    private Shooter shooter;
    private Inclinator inclinator;
    private Intake intake;
    private Kicker kick;
    private Aimer aimer;
    private PneumaticHub ph;
    private VisionSystem vision;
    private Amp amp;
    private SendableChooser<Command> autoChooser;

    // Controller Options
    private final String kXbox = "Xbox";
    private final String kFlight = "T16000M";
    private IDriveControls driveControls;
    private SendableChooser<String> driveControllerChooser = new SendableChooser<>();

    private String[] pdpList = {
        "LR DRV" , "LR TURN", "RR DRV", "RR TURN", "L SHT"  , "R SHT" ,
        "SHT ANG", "R CLIMB", "L INTK", "R MPM"  , "L MPM"  , "PH"    ,
        "L CLIMB", "R INTK" , "R ACC" , "L ACC"  , "RF TURN", "RF DRV",
        "LF TURN", "LR DRV" , "RIO"   , "22"     , "RADIO"  , "24"
    };

    public RobotContainer() {
        String serNum = RobotController.getSerialNumber();
        SmartDashboard.putString("Serial Number", serNum);
        //known Rio serial numbers:
        //031b525b = buzz
        //03064db7 = big buzz
        //031e3219 = practice chassis
        //03134cef = woody

        //subsystems used in all robots
        odometry = new Odometry();
        leds = new LedSubsystem(0, 92);
        vision = new VisionSystem(odometry); //not making variable as we won't change this subsystem

        //build the robot based on the Rio ID of the robot
        if (Robot.isSimulation() || (serNum.equals("031b525b")) || (serNum.equals("03064db7"))) {
            //either buzz or simulation
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
            shooter = new ShooterSim();
            intake = new IntakeSim();
            inclinator = new Inclinator(new InclinatorSim());
            kick = new KickerSim();
            aimer = new AimerSim();
            amp = new AmpSim();
        } else if (serNum.equals("031e3219")) {
            //practice robot
            swerveDrive = new SwerveDriveTrain(new PracticeSwerveHw(), odometry);
            odometry.setGyroHardware(new PigeonGyro(0));
            shooter = new ShooterSim();
            intake = new IntakeSim();
            inclinator = new Inclinator(new InclinatorSim());
            kick = new KickerSim();
            aimer = new AimerSim();
            amp = new AmpSim();
        } else if (serNum.equals("03134cef")) {
            //woody demo shooter
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
            shooter = new ShooterHw();
            intake = new IntakeHw();
            inclinator = new Inclinator(new InclinatorSim());
            kick = new KickerHw();
            aimer = new AimerSim();
            amp = new AmpSim();
        } else {
            //competition robot
            ph = new PneumaticHub();
            ph.enableCompressorAnalog(85, 115);
            Logger.RegisterSensor("Pressure", () -> ph.getPressure(0));

            Logger.RegisterPdp(new PowerDistribution(), pdpList);
            
            swerveDrive = new SwerveDriveTrain(new SwerveHw24(), odometry);
            odometry.setGyroHardware(new Pigeon2Gyro(0,kCanBusName));
            shooter = new ShooterHw();
            intake = new IntakeHw();
            inclinator = new Inclinator(new InclinatorHw());
            kick = new KickerHw();
            aimer = new AimerHw();
            amp = new AmpHw();
        }

        new DriverFeedback(vision, intake, leds);
        odometry.setSwerveDrive(swerveDrive);
        odometry.setStartingPose(new Pose2d(1.92, 2.79, new Rotation2d(0)));
        new RobotArbitrator(amp, inclinator, aimer);

        //add some buttons to press for development
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.WheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.WheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsDiamond()));
        SmartDashboard.putData("Test Leds", new TestLeds(leds));
        SmartDashboard.putData("Reset Wheel Position", new ResetWheelPosition(swerveDrive, odometry));
        SmartDashboard.putData("Pit Intake", intake.drive(() -> Intake.PIT_INTAKE_SPEED, false));
        SmartDashboard.putData("Home Climber", new HomeClimber(inclinator));
        SmartDashboard.putData("Test Aimer Low", aimer.setAimer(() -> 35));
        SmartDashboard.putData("Test Aimer High", aimer.setAimer(() -> 50));
        SmartDashboard.putData("Calibrate Shooter", new ShooterCalibrate(shooter, kick, aimer));
        SmartDashboard.putData("Auto Aim", new Autoshot(shooter, aimer, kick, odometry, intake, swerveDrive));
        SmartDashboard.putData("Swerve SysId Dynamic Forward", swerveDrive.sysIdDynamic(Direction.kForward));
        SmartDashboard.putData("Swerve SysId Dynamic Backward", swerveDrive.sysIdDynamic(Direction.kReverse));
        SmartDashboard.putData("Swerve SysId Quasistatic Forward", swerveDrive.sysIdQuasistatic(Direction.kForward));
        SmartDashboard.putData("Swerve SysId Quasistatic Backward", swerveDrive.sysIdQuasistatic(Direction.kReverse));

        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));
        NamedCommands.registerCommand("Intake", intake.drive(true));
        NamedCommands.registerCommand("Kick", intake.drive(false).withTimeout(0.75));
        NamedCommands.registerCommand("LightShot", new LightningFlash(leds, Color.kFirstRed));
        NamedCommands.registerCommand("StraightenWheels", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        NamedCommands.registerCommand("StartShooter", shooter.startShooter());
        //since simulation doesn't work with shooting yet, make this hack to timeout after 1.5 second of shooting
        if(Robot.isSimulation()) {
            NamedCommands.registerCommand("Shoot", new Autoshot(shooter, aimer, kick, odometry, intake, swerveDrive).withTimeout(1.5));
        } else {
            NamedCommands.registerCommand("Shoot", new AutoFixedShot(intake, shooter, kick, aimer));
        }

        // Controller chooser Setup
        driveControllerChooser.addOption("Xbox Controller", kXbox );
        driveControllerChooser.setDefaultOption("Fight Sticks", kFlight);
        SmartDashboard.putData("Drive Controller Select",driveControllerChooser);

        // Build an auto chooser. This will use Commands.none() as the default option.
        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.onChange(command -> {
            var path = command.getName();
            try {
                Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(path);
                odometry.setStartingPose(startingPose);
            } catch (RuntimeException e) {
                //if the path doesn't exist, don't change the starting position
                //this is common if the drivers select "None" (which would come as InstantCommand)
            }
        });
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        //setup commands that are used for driving based on starting controller
        if(driveControllerChooser.getSelected() == kFlight) {
            driveControls = new FlightDriveControls();
        }
        else{
            driveControls = new XboxDriveControls();
        }
        OperatorControls operatorControls = new OperatorControls();
        swerveDrive.setDefaultCommand(new DriveStick(swerveDrive, driveControls));
        swerveDrive.resetFieldOriented();
        OperatorStick operatorStick = new OperatorStick(shooter, operatorControls, kick, aimer, intake);
        leds.setDefaultCommand(new RainbowLeds(leds));
        //shooter.setDefaultCommand(operatorStick);
        //kick.setDefaultCommand(operatorStick);
        inclinator.setDefaultCommand(new DriveClimb(inclinator, operatorControls));
        aimer.setDefaultCommand(aimer.driveAimer(operatorControls::GetManualSubAim));
        new Trigger(operatorControls::IsIntakeRequested).whileTrue(intake.drive(false));
        new Trigger(operatorControls::IsIntakeDownRequested).whileTrue(intake.drive(false, true));
        new Trigger(driveControls::IsIntakeRequested).whileTrue(intake.drive(true));
        new Trigger(()->operatorControls.AutoSubAimRequested()).whileTrue(autoShot());
        new Trigger(operatorControls::IsCenterFieldShotRequested).whileTrue(shootAtTarget(TargetLocation.Corner));
        new Trigger(operatorControls::IsPillarShotRequested).whileTrue(shootAtTarget(TargetLocation.PillarFixed));
        new Trigger(operatorControls::IsAmpToggled).whileTrue(ampScore());
        new Trigger(operatorControls::ReverseShooterRequested).whileTrue(shooter.reverseShooter());
        new HomeClimber(inclinator).schedule();
    }

    public void disableBindings() {
        swerveDrive.removeDefaultCommand();
        shooter.removeDefaultCommand();
        kick.removeDefaultCommand();
        inclinator.removeDefaultCommand();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        configureAutoBuilder();
        return autoChooser.getSelected();
    }

    private void configureAutoBuilder() {
        // Configure the AutoBuilder
        AutoBuilder.configureHolonomic(
            odometry::getPose, // Robot pose supplier
            odometry::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(
                    UtilFunctions.getSetting("/PathPlanner/DriveP", 2), 
                    UtilFunctions.getSetting("/PathPlanner/DriveI", 0.5), 
                    UtilFunctions.getSetting("/PathPlanner/DriveD", 0)), // Translation PID constants
                new PIDConstants(
                    UtilFunctions.getSetting("/PathPlanner/TurnP", 1.5), 
                    UtilFunctions.getSetting("/PathPlanner/TurnI", 0.5), 
                    UtilFunctions.getSetting("/PathPlanner/TurnD", 0)), // Rotation PID constants
                3, // Max module speed, in m/s
                swerveDrive.getDriveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            odometry::shouldFlipAlliance, //shouldFlipPath Supplier that determines if paths should be flipped to the other side of the field. This will maintain a global blue alliance origin.
            swerveDrive // Reference to this subsystem to set requirements
        );
    }

    public Command shootAtTarget(TargetLocation location) {
        return new ParallelCommandGroup(
            shooter.runShooter(AutoShotLookup.getShooterSpeed(location)),
            kick.runKicker(AutoShotLookup.getKickerSpeed(location)),
            aimer.setAimer(AutoShotLookup.getShooterAngle(location))
        );
    }

    public Command ampScore() {
        return new ParallelCommandGroup(
            amp.setAmpDirection(true),
            shootAtTarget(TargetLocation.AmpFixed)
        ).withName("AmpShot");
    }

    public Command autoShot() {
        Trigger shotAimed = new Trigger(swerveDrive.aimedAtTarget()).and(shooter.atRpm());
        
        return new ParallelCommandGroup(
            shooter.startShooter(AutoShotLookup.getShooterSpeed(TargetLocation.Speaker)),
            swerveDrive.aimAtTargetBackwards(VisionSystem.getLocation(FieldLocation.Speaker))
        ).until(shotAimed)
        .withTimeout(3)
        .andThen(intake.drive(false).withTimeout(0.75))
        .finallyDo(shooter::stopShooter)
        .withName("AutoShot");
    }
}
