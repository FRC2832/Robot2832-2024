package frc.robot.hardware;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IShooterHw;

public class ShooterHw implements IShooterHw {
    private CANSparkFlex[] shooters;

    public ShooterHw() {
        shooters = new CANSparkFlex[4];
        shooters[0] = new CANSparkFlex(1, MotorType.kBrushless);
        shooters[1] = new CANSparkFlex(2, MotorType.kBrushless);
        shooters[2] = new CANSparkFlex(3, MotorType.kBrushless);
        shooters[3] = new CANSparkFlex(4, MotorType.kBrushless);

        shooters[1].setInverted(true);
        shooters[3].setInverted(true);
    }

    @Override
    public void setPower(double power) {
        for (CANSparkFlex shooter : shooters) {
            shooter.set(power);
        }
    }

    @Override
    public void setRpm(double rpm) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRpm'");
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Shooter Speed", shooters[0].getEncoder().getVelocity());
    }


}
