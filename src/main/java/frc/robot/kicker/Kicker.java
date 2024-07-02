package frc.robot.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Kicker extends SubsystemBase {
    public abstract void setPower(double power);
    public abstract void setRpm(double rpm);
    public abstract void updateInputs();
    public abstract double getCurrentRPM(int shooterID);

    public Kicker() {
        super();
    }
    
    @Override
    public void periodic() {
        updateInputs();
    }
    
    public void kick(double ShooterRPM) {
        setRpm(ShooterRPM*24/18);
    }
    public void stop() {
        setRpm(-10);
    }

    public double getRPM() {
        return 0.00;
    }
    
    public static double RPMToVelocity(double RPM) {
        return ((((RPM*(Math.PI)*2)/60)*2)/(39.3700787402));
    }

    public boolean isAtSpeed() {
        return false;
    }
   }