package org.livoniawarriors;

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
}