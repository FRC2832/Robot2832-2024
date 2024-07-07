package frc.robot.aimer;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class AimerSim extends Aimer {
    private DoubleSolenoid.Value direction;
    private double angle;
    private SimDevice aimerDevice;
    private SimDouble aimerAngle;
    private final double kAnglePerStep = 2;

    public AimerSim() {
        direction = Value.kOff;
        angle = 37;
        aimerDevice = SimDevice.create("Jukebox:Aimer");
        if(aimerDevice != null) {
            aimerAngle = aimerDevice.createDouble("Angle", Direction.kOutput, angle);
        }
    }

    @Override
    public void updateInputs() {
        if (direction == Value.kForward) {
            angle = Math.min(angle+kAnglePerStep,75);
        } else if (direction == Value.kReverse) {
            angle = Math.max(angle-kAnglePerStep,22);
        } else {
            //do nothing because it was stopped
        }

        if(aimerDevice != null) {
            aimerAngle.set(angle);
        }
    }

    @Override
    public double getAngle() {
        return angle;
    }

    @Override
    public void driveUp() {
        direction = Value.kReverse;
    }

    @Override
    public void driveDown() {
        direction = Value.kForward;
    }

    @Override
    public void stop() {
        direction = Value.kOff;
    }

    @Override
    public void startPulse(double time, boolean goingUp) {
        // TODO Auto-generated method stub
    }
    
}
