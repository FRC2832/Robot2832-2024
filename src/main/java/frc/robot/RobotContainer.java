// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;
import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.odometry.PigeonGyro;
import org.livoniawarriors.odometry.SimSwerveGyro;
import org.livoniawarriors.swerve.MoveWheels;
import org.livoniawarriors.swerve.SwerveDriveSim;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controls.FlightDriveControls;
import frc.robot.Controls.OperatorControls;
import frc.robot.Controls.XboxDriveControls;
import frc.robot.commands.DriveStick;
import frc.robot.commands.DriveAimer;
import frc.robot.commands.DriveClimb;
import frc.robot.commands.OperatorStick;
import frc.robot.commands.ResetWheelPosition;
import frc.robot.hardware.IntakeHw;
import frc.robot.hardware.ShooterHw;
import frc.robot.hardware.KickerHw;
import frc.robot.hardware.PneumaticHW;
import frc.robot.hardware.PracticeSwerveHw;
import frc.robot.interfaces.IDriveControls;
import frc.robot.simulation.InclinatorSim;
import frc.robot.simulation.IntakeSim;
import frc.robot.simulation.KickerSim;
import frc.robot.simulation.PneumaticsSim;
import frc.robot.simulation.ShooterSim;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Inclinator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Pneumatics;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    private SwerveDriveTrain swerveDrive;
    private Odometry odometry;
    private LedSubsystem leds;
    private Shooter shooter;
    private Inclinator inclinator;
    private Intake intake;
    private Kicker kick;
    private Pneumatics aimer;
    private PneumaticHub ph;
    private SendableChooser<Command> autoChooser;

    // Controller Options
    private final String kXbox = "Xbox";
    private final String kFlight = "T16000M";
    private IDriveControls driveControls;
    private OperatorControls operatorControls;
    private SendableChooser<String> driveControllerChooser = new SendableChooser<>();

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
        leds = new LedSubsystem(0, 10);
        new VisionSystem(odometry); //not making variable as we won't change this subsystem

        //build the robot based on the Rio ID of the robot
        if (Robot.isSimulation() || (serNum.equals("031b525b")) || (serNum.equals("03064db7"))) {
            //either buzz or simulation
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
            shooter = new Shooter(new ShooterSim());
            intake = new Intake(new IntakeSim());
            inclinator = new Inclinator(new InclinatorSim());
            kick = new Kicker(new KickerSim());
            aimer = new Pneumatics(new PneumaticsSim());
        } else if (serNum.equals("031e3219")) {
            //practice robot
            swerveDrive = new SwerveDriveTrain(new PracticeSwerveHw(), odometry);
            odometry.setGyroHardware(new PigeonGyro(0));
            shooter = new Shooter(new ShooterSim());
            intake = new Intake(new IntakeSim());
            inclinator = new Inclinator(new InclinatorSim());
            kick = new Kicker(new KickerSim());
            aimer = new Pneumatics(new PneumaticsSim());
        } else if (serNum.equals("03134cef")) {
            //woody demo shooter
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
            shooter = new Shooter(new ShooterHw());
            intake = new Intake(new IntakeHw());
            //inclinator = new Inclinator(new InclinatorSim());
            kick = new Kicker(new KickerHw());
            aimer = new Pneumatics(new PneumaticsSim());
        } else {
            //competition robot
            ph = new PneumaticHub();
            ph.enableCompressorAnalog(50, 80);

            swerveDrive = new SwerveDriveTrain(new PracticeSwerveHw(), odometry);
            odometry.setGyroHardware(new PigeonGyro(0));
            shooter = new Shooter(new ShooterSim());
            intake = new Intake(new IntakeSim());
            inclinator = new Inclinator(new InclinatorSim());
            kick = new Kicker(new KickerSim());
            aimer = new Pneumatics(new PneumaticHW());
        }

        odometry.setSwerveDrive(swerveDrive);
        odometry.setStartingPose(new Pose2d(1.92, 2.79, new Rotation2d(0)));

        //add some buttons to press for development
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.WheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.WheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.WheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.DriveWheelsDiamond()));
        SmartDashboard.putData("Test Leds", new TestLeds(leds));
        SmartDashboard.putData("Reset Wheel Position", new ResetWheelPosition(swerveDrive, odometry));

        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));
        // Need a shoot command in the future to shoot with

        // Controller chooser Setup
        driveControllerChooser.addOption("Xbox Controller", kXbox );
        driveControllerChooser.setDefaultOption("Fight Sticks", kFlight);
        SmartDashboard.putData("Drive Controller Select",driveControllerChooser);
        // Configure the AutoBuilder
        AutoBuilder.configureHolonomic(
            odometry::getPose, // Robot pose supplier
            odometry::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                swerveDrive.getMaxSpeed(), // Max module speed, in m/s
                swerveDrive.getDriveBaseRadius(), // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            odometry::shouldFlipAlliance, //shouldFlipPath Supplier that determines if paths should be flipped to the other side of the field. This will maintain a global blue alliance origin.
            swerveDrive // Reference to this subsystem to set requirements
        );

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        OperatorStick operatorStick = new OperatorStick(shooter, operatorControls, kick);
        leds.setDefaultCommand(new RainbowLeds(leds));
        shooter.setDefaultCommand(operatorStick);
        kick.setDefaultCommand(operatorStick);
        inclinator.setDefaultCommand(new DriveClimb(inclinator));
        new Trigger(() -> Math.abs(operatorControls.GetManualSubAim()) > 0.2).whileTrue(new DriveAimer(operatorControls, aimer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
