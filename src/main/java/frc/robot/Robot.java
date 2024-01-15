// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.GitVersion;
import org.livoniawarriors.Logger;

import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Logger logger;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        //display the Git info for the build in the network tables
        GitVersion.loadVersion().printVersions();

        //internal logger class
        logger = new Logger();
        Logger.RegisterLoopTimes(this);
        SmartDashboard.putData(CommandScheduler.getInstance());

        /* This is a fix from 2023 with a Rio2.  We took a very hard hit and the Rio went into brownout
         * protection.  That was fine.  But we stayed stuck in brownout protection throughout power cycles
         * and code deploys.  We never figured out how to clear brownout before the match and missed a
         * playoff match because of it.  The best guess we have was to take out and put back in the SD card,
         * or put a different SD card in the robot, as this cleared some internal FPGA flag.
         * 
         * This specific change just lowers the threshold the Rio2 goes into brownout to 3V.  Because the
         * Rio2 needs 5.5V to run, the hardware will shutoff before brownout protection happens, and we
         * should never see Brownout Protection again.
         */
        RobotController.setBrownoutVoltage(3);

        //turn off hot reload at the competition.
        if (DriverStation.isFMSAttached()) {
            PPLibTelemetry.enableCompetitionMode();
        }

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        m_robotContainer.configureBindings();

        //start logging class after all the subsystems have initialized
        logger.start();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        //this clears all the old polled triggers
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        //since we sometimes switch configurations based on controller type, 
        //reset the bindings
        m_robotContainer.configureBindings();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
