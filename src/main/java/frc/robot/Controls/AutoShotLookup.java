package frc.robot.Controls;

import org.livoniawarriors.UtilFunctions;

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
        if(isCenterFieldShot) {
            this.shooterSpeed = UtilFunctions.getSetting("/Shooting/CenterFieldShooterSpeed", 4300.);
            this.kickerSpeed = UtilFunctions.getSetting("/Shooting/CenterFieldKickerSpeed", 4500.);
            this.angle = UtilFunctions.getSetting("/Shooting/CenterFieldShooterAngle", 51.);
        } else {
            this.shooterSpeed = UtilFunctions.getSetting("/Shooting/PillarShooterSpeed", 4500.);
            this.kickerSpeed = UtilFunctions.getSetting("/Shooting/PillarKickerSpeed", 4500.);
            this.angle = UtilFunctions.getSetting("/Shooting/PillarShooterAngle", 40.);
        }
    }

    public void printValues() {
        SmartDashboard.putNumber("Auto Angle", angle);
        SmartDashboard.putNumber("Auto Kicker", kickerSpeed);
        SmartDashboard.putNumber("Auto Shot RPM", shooterSpeed);
    }
}
