package frc.robot.Controls;

import java.util.function.DoubleSupplier;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.VisionSystem;
import frc.robot.VisionSystem.FieldLocation;

public class AutoShotLookup {
    private double angle;
    private double kickerSpeed;
    private double shooterSpeed;

    private static InterpolatingDoubleTreeMap speed, angleMap, kicker;

    public static enum TargetLocation {
        AmpFixed,
        SpeakerFixed,
        PillarFixed,
        CenterFixed,
        Corner,
        Speaker
    }

    static {
        speed = new InterpolatingDoubleTreeMap();
        angleMap = new InterpolatingDoubleTreeMap();
        kicker = new InterpolatingDoubleTreeMap();

        //input: Meters from target, output: RPM of shot  
        speed.put(1.375, 5000.);
        speed.put(1.78, 5000.);
        speed.put(2.32, 5000.);
        speed.put(2.8, 5000.);
        speed.put(3.12, 5300.);
        speed.put(3.43, 5300.);
        speed.put(3.92, 5500.);

        //input: Meters from target, output: Degrees of shot  
        angleMap.put(1.375, 54.);
        angleMap.put(1.78, 50.);
        angleMap.put(2.32, 46.);
        angleMap.put(2.8, 39.);
        angleMap.put(3.12, 36.);
        angleMap.put(3.43, 32.);
        angleMap.put(3.92, 28.);

        //input: Meters from target, output: RPM for kicker
        kicker.put(1.375, 6000.);
        kicker.put(1.78, 6000.);
        kicker.put(2.32, 6000.);
        kicker.put(2.8, 6000.);
        kicker.put(3.12, 6000.);
        kicker.put(3.43, 6000.);
        kicker.put(3.92, 6000.);
    }

    public AutoShotLookup (double distance) {
        this.angle = angleMap.get(distance);
        this.kickerSpeed = kicker.get(distance);
        this.shooterSpeed = speed.get(distance);
    }

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

    public static DoubleSupplier getShooterAngle(TargetLocation location) {
        switch(location) {
            case AmpFixed:
                return UtilFunctions.getSettingSub("/Shooting/AmpShotAngle", 54.);
            case PillarFixed:
                return UtilFunctions.getSettingSub("/Shooting/PillarShooterAngle", 40.);
            case SpeakerFixed:
                return UtilFunctions.getSettingSub("/Shooting/SpeakerShooterAngle", 54.);
            case CenterFixed:
                return UtilFunctions.getSettingSub("/Shooting/CenterFieldShooterAngle", 54.);
            case Speaker:
            case Corner:
            default:
                return () -> angleMap.get(VisionSystem.getDistanceToTarget(FieldLocation.Speaker));
        }
    }

    public static DoubleSupplier getShooterSpeed(TargetLocation location) {
        switch(location) {
            case AmpFixed:
                return UtilFunctions.getSettingSub("/Shooting/AmpShotSpeed", 450.);
            case PillarFixed:
                return UtilFunctions.getSettingSub("/Shooting/PillarShooterSpeed", 4500.);
            case SpeakerFixed:
                return UtilFunctions.getSettingSub("/Shooting/SpeakerShooterSpeed", 6000.);
            case CenterFixed:
                return UtilFunctions.getSettingSub("/Shooting/CenterFieldShooterSpeed", 4500.);
            case Speaker:
            case Corner:
            default:
                return () -> speed.get(VisionSystem.getDistanceToTarget(FieldLocation.Speaker));
        }
    }

    public static DoubleSupplier getKickerSpeed(TargetLocation location) {
        switch(location) {
            case AmpFixed:
                return UtilFunctions.getSettingSub("/Shooting/AmpKickerSpeed", 450.);
            case PillarFixed:
                return UtilFunctions.getSettingSub("/Shooting/PillarKickerSpeed", 4500.);
            case SpeakerFixed:
                return UtilFunctions.getSettingSub("/Shooting/SpeakerKickerSpeed", 6500.);
            case CenterFixed:
                return UtilFunctions.getSettingSub("/Shooting/CenterFieldKickerSpeed", 4500.);
            case Speaker:
            case Corner:
            default:
                return () -> kicker.get(VisionSystem.getDistanceToTarget(FieldLocation.Speaker));
        }
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
