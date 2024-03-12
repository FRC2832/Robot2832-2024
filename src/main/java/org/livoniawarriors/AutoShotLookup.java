package org.livoniawarriors;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoShotLookup {
    private double angle;
    private double kickerSpeed;
    private double shooterSpeed;

    public AutoShotLookup(double angle, double kickerSpeed, double shooterSpeed) {
        this.angle = angle;
        this.kickerSpeed = kickerSpeed;
        this.shooterSpeed = shooterSpeed;
    }

    public void setValues(double angle, double kickerSpeed, double shooterSpeed) {
        this.angle = angle;
        this.kickerSpeed = kickerSpeed;
        this.shooterSpeed = shooterSpeed;
    }

    public double getAngle() {
        return angle;
    }

    public double getKickerSpeed() {
        return kickerSpeed;
    }

    public double getShooterSpeed() {
        return shooterSpeed;
    }

    public void getPrefs(boolean isCenterFieldShot) {
        this.shooterSpeed = isCenterFieldShot ? Preferences.getDouble("/Preferences/Shooting/CenterFieldShooterSpeed", 4300) : Preferences.getDouble("/Preferences/Shooting/PillarShooterSpeed", 4500);
        this.kickerSpeed = isCenterFieldShot ? Preferences.getDouble("/Preferences/Shooting/CenterFieldKickerSpeed", 4500) : Preferences.getDouble("/Preferences/Shooting/PillarKickerSpeed", 4500);
        this.angle = isCenterFieldShot ? Preferences.getDouble("/Preferences/Shooting/CenterFieldShooterAngle", 51) : Preferences.getDouble("/Preferences/Shooting/PillarShooterAngle", 40);
    }

    public void printValues() {
        SmartDashboard.putNumber("Auto Angle", angle);
        SmartDashboard.putNumber("Auto Kicker", kickerSpeed);
        SmartDashboard.putNumber("Auto Shot RPM", shooterSpeed);
    }
}
