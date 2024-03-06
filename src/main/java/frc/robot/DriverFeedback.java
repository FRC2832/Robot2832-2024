package frc.robot;

import org.livoniawarriors.Logger;
import org.livoniawarriors.REVDigitBoard;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.intake.Intake;

public class DriverFeedback extends SubsystemBase {
    private REVDigitBoard digit;
    private StringPublisher ronyMessage;
    private VisionSystem vision;
    private Intake intake;
    private LedSubsystem leds;
    private boolean lastPiece;
    private static Color commandColor;

    public DriverFeedback(VisionSystem vision, Intake intake, LedSubsystem leds) {
        digit = new REVDigitBoard();
        this.vision = vision;
        this.intake = intake;
        this.leds = leds;
        commandColor = Color.kBlack;
        ronyMessage = UtilFunctions.getNtPub("/DriverFeedback/RonyMessage", "RONY");
    }

    @Override
    public void periodic() {

        String message;
        //run the digit board
        if (!Logger.IsFlashDriveAttached()) {
            message = "FLSH";
        } else if(Logger.FaultSet()) {
            message = " FLT";
        } else if(!vision.isCameraPresent()) {
            message = " PI ";
        } else if (Logger.StickyFaultSet()) {
            message = "SFLT";
        } else {
            message = "RONY";
        }
        digit.display(message);
        ronyMessage.set(message);

        //flash orange if we have a note
        boolean pieceDetected = intake.isPieceDetected();
        if(lastPiece == false && pieceDetected) {
            CommandScheduler.getInstance().schedule(new LightningFlash(leds, Color.kOrange));
        } else if(commandColor != Color.kBlack) {
            for(int i=0; i<leds.getLength(); i++) {
                leds.setLed(0, commandColor);
            }
        }
        lastPiece = pieceDetected;
    }

    public static void setColor(Color color) {
        commandColor = color;
    }
}
