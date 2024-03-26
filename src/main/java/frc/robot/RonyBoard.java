package frc.robot;
import org.livoniawarriors.Logger;
import org.livoniawarriors.REVDigitBoard;

public class RonyBoard{
    private REVDigitBoard digit;

    public RonyBoard(){
        digit = new REVDigitBoard();
    }
    public void update() {

        //run the digit board
        if(Logger.FaultSet()) {
            digit.display(" FLT");
        } else if (Logger.StickyFaultSet()) {
            digit.display("SFLT");
        } else {
            digit.display("RONY");
        }
        /*
        if(digit.getButtonA()) {
            Logger.checkClearFaults(true);
        } */
    }
}