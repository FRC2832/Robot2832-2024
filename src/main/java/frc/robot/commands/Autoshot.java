package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

public class Autoshot {
    public boolean shoot(double velocity, double spin, double angle){
        
        
        return true;
    } 
    //I am assuming this is what was meant by a lookup table. Ideally, given x and y difference, it would calculate
    //closest set value and return angle x and angle y
    public double[] estimate(double[] pos){
        Map<double[], double[]> map = new HashMap<double[], double[]>();
        map.put(new double[]{0.0,0.0}, new double[]{0.0,0.0});
        map.put(new double[]{0.0,0.0}, new double[]{0.0,0.0});
        map.put(new double[]{0.0,0.0}, new double[]{0.0,0.0});
        map.put(new double[]{0.0,0.0}, new double[]{0.0,0.0});
        map.put(new double[]{0.0,0.0}, new double[]{0.0,0.0});
        map.put(new double[]{0.0,0.0}, new double[]{0.0,0.0});
        double[] closestKey = new double[] {10000.0,10000.0};
        double minDistance=1000;
        for (double[] key : map.keySet()){
            if(minDistance>Math.pow(Math.pow(key[0]-pos[0],2)+Math.pow(key[1]-pos[1],2),0.5)){
//Hypotenuse distance based on x&y^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                closestKey = key;
            }
        }
        return map.get(closestKey);
    }
// This is the fun part
    public double[] calculate(){
        double [] result = {0.0,0.0};



        return result;
    }
}
