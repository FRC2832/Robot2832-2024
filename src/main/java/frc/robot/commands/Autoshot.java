package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import frc.robot.hardware.PneumaticHW;
import frc.robot.interfaces.IPneumaticHW;
import edu.wpi.first.wpilibj2.command.Command;

public class Autoshot extends Command{
    private Shooter shooter;
    public double target=-1;
    public Autoshot(Shooter shooter){
        this.shooter = shooter;
    }
    public double[] calculate(){
        double [] result = {0.0,0.0};



        return result;
    }
    @Override 
    public void execute(){
        double target=shooter.estimate(0)[1];//HOW TO GET DISTANCE
        if(this.target!=1){
        if(this.target==target){
            IPneumaticHW.stop(this.target);
        }else{
            IPneumaticHW.start(this.target>target);
        }}


        


    }
}
