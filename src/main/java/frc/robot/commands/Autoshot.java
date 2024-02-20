package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;

public class Autoshot extends Command {
    Shooter shooter;
    Pneumatics pneumatic;
    
    public Autoshot(Shooter shooter, Pneumatics pneumatic){
        this.shooter = shooter;
        this.pneumatic = pneumatic;
    }

    //TODO: Physics based approach TBD
    public double[] calculate() {
        double [] result = {0.0,0.0};
        return result;
    }
    
    @Override 
    public void execute(){
        double targetAngle = shooter.estimate(0)[1]; //Get angle from hashmap, HOW TO GET Distance
        //^^WE NEED TO PASS TARGETANGLE AS -1 when we need it idle.
        pneumatic.goTo(targetAngle);
    }
}