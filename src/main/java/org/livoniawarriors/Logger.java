package org.livoniawarriors;

import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.MotControllerJNI;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStickyFaults;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_Faults;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;

@SuppressWarnings("removal")
public class Logger implements Runnable {
    private final double VOLTS_PER_PSI = 1.931/100; //2.431V at 100psi

    private static HashMap<String,Object> items = new HashMap<String,Object>();
    private static PowerDistribution pdp;
    private static String[] pdpNames;
    private static PneumaticHub ph;
    private static String[] pneumaticNames;
    private static BasePigeon pigeon;

    //used to stop warning about not closing motor, since we really don't...
    private static CANSparkMax spark;       
    private static CANSparkFlex sparkFlex;

    private NetworkTable currentTable;
    private NetworkTable commandTable;
    private NetworkTable tempTable;
    private NetworkTable faultTable;
    private NetworkTable stickyTable;
    private NetworkTable sensorTable;
    private NetworkTable canStatusTable;
    private static NetworkTable taskTimings;

    private BooleanPublisher flashDrivePresent;

    private static boolean faultSet;
    private static boolean sfaultSet;
    private static boolean flashDriveAttached;

    //solenoids, compressor

    public Logger() {
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        var log = DataLogManager.getLog();
        DriverStation.startDataLog(log);

        //create our logging table references
        canStatusTable = NetworkTableInstance.getDefault().getTable("CAN Status");
        currentTable = NetworkTableInstance.getDefault().getTable("Motor Currents");
        tempTable = NetworkTableInstance.getDefault().getTable("Motor Temps");
        commandTable = NetworkTableInstance.getDefault().getTable("Device Commands");
        faultTable = NetworkTableInstance.getDefault().getTable("Device Faults");
        stickyTable = NetworkTableInstance.getDefault().getTable("Device Sticky Faults");
        sensorTable = NetworkTableInstance.getDefault().getTable("Sensors");
        taskTimings = NetworkTableInstance.getDefault().getTable("Task Timings (ms)");
        SmartDashboard.putBoolean("Clear Faults", false);

        // Set the scheduler to log Shuffleboard events for command initialize,
        // interrupt, finish
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        command -> Shuffleboard.addEventMarker(
                                "Command initialized", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        command -> Shuffleboard.addEventMarker(
                                "Command interrupted", command.getName(), EventImportance.kNormal));
        CommandScheduler.getInstance()
                .onCommandFinish(
                        command -> Shuffleboard.addEventMarker(
                                "Command finished", command.getName(), EventImportance.kNormal));

        flashDrivePresent = UtilFunctions.getNtPub("/Sensors/Flash Drive Attached", false);
    }

    public void start() {
        //we run at 100ms rate because of all the CAN traffic, especially with the PDP log channels
        UtilFunctions.addPeriodic(this, 0.1, 0);
    }

    public static void RegisterTalon(String name, BaseTalon talon) {
        items.put(name, talon);
    }

    public static void RegisterTalon(String name, CoreTalonFX talon) {
        items.put(name, talon);
    }

    public static void RegisterCanSparkMax(String name, CANSparkMax spark) {
        items.put(name, spark);
    }

    public static void RegisterCanSparkFlex(String name, CANSparkFlex spark) {
        items.put(name, spark);
    }

    public static void RegisterCanCoder(String name, CANCoder coder) {
        items.put(name, coder);
    }

    public static void RegisterSensor(String name, DoubleSupplier value) {
        items.put(name, value);
    }

    public static void RegisterPdp(PowerDistribution pdp, String[] channelNames) {
        Logger.pdp = pdp;
        pdpNames = channelNames;
    }

    public static void RegisterPneumaticHub(PneumaticHub ph, String[] channelNames) {
        Logger.ph = ph;
        pneumaticNames = channelNames;
    }

    public static void RegisterLoopTimes(Robot robot) {
        new LoopTimeLogger(robot, taskTimings);
    }

    public static void RegisterPigeon(BasePigeon pigeon) {
        Logger.pigeon = pigeon;
    }

    public void run() {
        // Print keys
        for (String i : items.keySet()) {
            var item = items.get(i);

            if(item instanceof BaseTalon) {
                readTalon(i, (BaseTalon)item);
            } else if(item instanceof CoreTalonFX) {
                readTalon(i, (CoreTalonFX)item);
            } else if(item instanceof DoubleSupplier) {
                sensorTable.getEntry(i).setDouble(((DoubleSupplier)item).getAsDouble());
            } else if(item instanceof CANCoder) {
                var coder = (CANCoder)item;
                sensorTable.getEntry(i + " Angle").setDouble(coder.getAbsolutePosition());
                sensorTable.getEntry(i + " Mag Str").setString(coder.getMagnetFieldStrength().name());

                var faults = new CANCoderFaults();
                coder.getFaults(faults);
                faultTable.getEntry(i).setString(readFaultStruct(faults));

                var sFaults = new CANCoderStickyFaults();
                coder.getStickyFaults(sFaults);
                stickyTable.getEntry(i).setString(readFaultStruct(sFaults));
                canStatusTable.getEntry(i).setString(coder.getLastError().name());
            } else if(item instanceof CANSparkMax) {
                spark = (CANSparkMax)item;

                commandTable.getEntry(i).setDouble(spark.getAppliedOutput()*spark.getBusVoltage());
                currentTable.getEntry(i).setDouble(spark.getOutputCurrent());
                faultTable.getEntry(i).setString(readSparkFaults(spark.getFaults()));
                stickyTable.getEntry(i).setString(readSparkFaults(spark.getStickyFaults()));
                tempTable.getEntry(i).setDouble(spark.getMotorTemperature());
                canStatusTable.getEntry(i).setString(spark.getLastError().name());
            } else if(item instanceof CANSparkFlex) {
                sparkFlex = (CANSparkFlex)item;

                commandTable.getEntry(i).setDouble(sparkFlex.getAppliedOutput()*sparkFlex.getBusVoltage());
                currentTable.getEntry(i).setDouble(sparkFlex.getOutputCurrent());
                faultTable.getEntry(i).setString(readSparkFaults(sparkFlex.getFaults()));
                stickyTable.getEntry(i).setString(readSparkFaults(sparkFlex.getStickyFaults()));
                tempTable.getEntry(i).setDouble(sparkFlex.getMotorTemperature());
                canStatusTable.getEntry(i).setString(sparkFlex.getLastError().name());
            } else {
                //unknown table
            }
        }

        if(pdp != null) {
            for(int i=0; i<pdpNames.length; i++) {
                if(pdpNames[i] != null) {
                    currentTable.getEntry("PDP Current " + pdpNames[i]).setDouble(pdp.getCurrent(i));
                } else {
                }
            }
            sensorTable.getEntry("PDP Voltage").setDouble(pdp.getVoltage());
            sensorTable.getEntry("PDP Total Current").setDouble(pdp.getTotalCurrent());
            tempTable.getEntry("PDP").setDouble(pdp.getTemperature());
            faultTable.getEntry("PDP").setString(readFaultStruct(pdp.getFaults()));
            stickyTable.getEntry("PDP").setString(readFaultStruct(pdp.getStickyFaults()));
            //no CAN status to report
        }

        if(ph != null) {
            var solenoids = ph.getSolenoids();
            for(int i=0; i<pneumaticNames.length; i++) {
                if(pneumaticNames[i] != null) {
                    commandTable.getEntry(pneumaticNames[i]).setBoolean((solenoids & (1 << i)) == 1);
                }
            }
            var volts = ph.getAnalogVoltage(0);
            sensorTable.getEntry("Pressure Sensor").setDouble((volts - 0.5) / VOLTS_PER_PSI);
            sensorTable.getEntry("Pressure Sensor Voltage").setDouble(volts);
            currentTable.getEntry("Compressor").setDouble(ph.getCompressorCurrent());
            currentTable.getEntry("Solenoids").setDouble(ph.getSolenoidsTotalCurrent());
            faultTable.getEntry("PH").setString(readFaultStruct(ph.getFaults()));
            stickyTable.getEntry("PH").setString(readFaultStruct(ph.getStickyFaults()));
            //no CAN status to report
        }

        if(pigeon != null) {
            double[] pigeon_data = new double[3];
            pigeon.getYawPitchRoll(pigeon_data);
            sensorTable.getEntry("Pigeon Yaw").setDouble(pigeon_data[0]);
            sensorTable.getEntry("Pigeon Pitch").setDouble(pigeon_data[1]);
            sensorTable.getEntry("Pigeon Roll").setDouble(pigeon_data[2]);

            short[] accel_data = new short[3];
            pigeon.getBiasedAccelerometer(accel_data);
            sensorTable.getEntry("Pigeon Ax").setDouble(accel_data[0]/16384.);
            sensorTable.getEntry("Pigeon Ay").setDouble(accel_data[1]/16384.);
            sensorTable.getEntry("Pigeon Az").setDouble(accel_data[2]/16384.);

            canStatusTable.getEntry("Pigeon").setString(pigeon.getLastError().name());

            if (pigeon instanceof PigeonIMU) {
                var p1 = (PigeonIMU)pigeon;
                PigeonIMU_Faults p1Faults = new PigeonIMU_Faults();
                p1.getFaults(p1Faults);
                faultTable.getEntry("Pigeon").setString(readFaultStruct(p1Faults));

                p1.getStickyFaults(p1Faults);
                stickyTable.getEntry("Pigeon").setString(readFaultStruct(p1Faults));
            } else if (pigeon instanceof Pigeon2) {
                var p2 = (Pigeon2)pigeon;
                Pigeon2_Faults p2Faults = new Pigeon2_Faults();
                p2.getFaults(p2Faults);
                faultTable.getEntry("Pigeon").setString(readFaultStruct(p2Faults));

                p2.getStickyFaults(p2Faults);
                stickyTable.getEntry("Pigeon").setString(readFaultStruct(p2Faults));
            } else {
                //unknown pigeon
            }
        }

        var canStatus = RobotController.getCANStatus();
        canStatusTable.getEntry("CAN Bandwidth").setDouble(canStatus.percentBusUtilization);
        canStatusTable.getEntry("CAN Bus Off Count").setDouble(canStatus.busOffCount);
        canStatusTable.getEntry("CAN RX Error Count").setDouble(canStatus.receiveErrorCount);
        canStatusTable.getEntry("CAN Tx Error Count").setDouble(canStatus.transmitErrorCount);
        canStatusTable.getEntry("CAN Tx Full Count").setDouble(canStatus.txFullCount);

        sensorTable.getEntry("Rio 3.3V Voltage").setDouble(RobotController.getVoltage3V3());
        sensorTable.getEntry("Rio 5V Voltage").setDouble(RobotController.getVoltage5V());
        sensorTable.getEntry("Rio 6V Voltage").setDouble(RobotController.getVoltage6V());
        sensorTable.getEntry("Rio 3.3V Current").setDouble(RobotController.getCurrent3V3());
        sensorTable.getEntry("Rio 5V Current").setDouble(RobotController.getCurrent5V());
        sensorTable.getEntry("Rio 6V Current").setDouble(RobotController.getCurrent6V());
        sensorTable.getEntry("Rio CPU Temp").setDouble(RobotController.getCPUTemp());

        checkClearFaults(false);

        Set<String> keys0 = faultTable.getKeys();
        Set<String> stickyKeys0 = stickyTable.getKeys();

        String[] keys = new String[keys0.size()];
        String[] stickyKeys = new String[stickyKeys0.size()];
        
        keys0.toArray(keys);
        stickyKeys0.toArray(stickyKeys);
        
        faultSet = false;
        for(String i: keys) {
            var faultName = faultTable.getEntry(i).getString("EROR");
            if(!faultName.equals("Ok") && !faultName.equals("Not Implemented")) {
                faultSet = true;
            }
        }
        
        sfaultSet = false;
        for(String i: stickyKeys) {
            var faultName = stickyTable.getEntry(i).getString("EROR");
            if(!faultName.equals("Ok") && !faultName.equals("Not Implemented")) {
                sfaultSet = true;
            }
        }

        if (Robot.isReal()) {
            flashDriveAttached = Files.exists(Paths.get("/u"));
        } else {
            flashDriveAttached = true;
        }
        flashDrivePresent.set(flashDriveAttached);
    }

    private void readTalon(String name, CoreTalonFX talon) {
        var current = talon.getSupplyCurrent();
        commandTable.getEntry(name).setDouble(talon.getMotorVoltage().getValueAsDouble());
        currentTable.getEntry(name).setDouble(current.getValueAsDouble());
        faultTable.getEntry(name).setString(readTalonFaults(talon));
        stickyTable.getEntry(name).setString(readStickyTalonFaults(talon));
        tempTable.getEntry(name).setDouble(talon.getDeviceTemp().getValueAsDouble());
        canStatusTable.getEntry(name).setString(current.getStatus().getName());
    }

    private String readTalonFaults(CoreTalonFX talon) {
        String msg = "";
        if(talon.getFault_BootDuringEnable().getValue())
            msg += "BootDuringEnable ";
        if(talon.getFault_BridgeBrownout().getValue())
            msg += "BridgeBrownout ";
        if(talon.getFault_DeviceTemp().getValue())
            msg += "DeviceTemp ";
        if(talon.getFault_ForwardHardLimit().getValue())
            msg += "ForwardHardLimit ";
        if(talon.getFault_ForwardSoftLimit().getValue())
            msg += "ForwardSoftLimit ";
        if(talon.getFault_FusedSensorOutOfSync().getValue())
            msg += "FusedSensorOutOfSync ";
        if(talon.getFault_Hardware().getValue())
            msg += "Hardware ";

        if(talon.getFault_MissingDifferentialFX().getValue())
            msg += "MissingDifferentialFX ";
        if(talon.getFault_OverSupplyV().getValue())
            msg += "OverSupplyV ";
        if(talon.getFault_ProcTemp().getValue())
            msg += "ProcTemp ";
        if(talon.getFault_RemoteSensorDataInvalid().getValue())
            msg += "RemoteSensorDataInvalid ";
        if(talon.getFault_RemoteSensorPosOverflow().getValue())
            msg += "RemoteSensorPosOverflow ";
        if(talon.getFault_RemoteSensorReset().getValue())
            msg += "RemoteSensorReset ";
        if(talon.getFault_ReverseHardLimit().getValue())
            msg += "ReverseHardLimit ";

        if(talon.getFault_ReverseSoftLimit().getValue())
            msg += "ReverseSoftLimit ";
        if(talon.getFault_StatorCurrLimit().getValue())
            msg += "StatorCurrLimit ";
        if(talon.getFault_SupplyCurrLimit().getValue())
            msg += "SupplyCurrLimit ";
        if(talon.getFault_Undervoltage().getValue())
            msg += "Undervoltage ";
        if(talon.getFault_UnlicensedFeatureInUse().getValue())
            msg += "UnlicensedFeatureInUse ";
        if(talon.getFault_UnstableSupplyV().getValue())
            msg += "UnstableSupplyV ";
        if(talon.getFault_UsingFusedCANcoderWhileUnlicensed().getValue())
            msg += "UsingFusedCANcoderWhileUnlicensed ";
        
        if(msg == "") {
            msg = "Ok";
        }
        return msg;
    }

    private String readStickyTalonFaults(CoreTalonFX talon) {
        String msg = "";
        if(talon.getStickyFault_BootDuringEnable().getValue())
            msg += "BootDuringEnable ";
        if(talon.getStickyFault_BridgeBrownout().getValue())
            msg += "BridgeBrownout ";
        if(talon.getStickyFault_DeviceTemp().getValue())
            msg += "DeviceTemp ";
        if(talon.getStickyFault_ForwardHardLimit().getValue())
            msg += "ForwardHardLimit ";
        if(talon.getStickyFault_ForwardSoftLimit().getValue())
            msg += "ForwardSoftLimit ";
        if(talon.getStickyFault_FusedSensorOutOfSync().getValue())
            msg += "FusedSensorOutOfSync ";
        if(talon.getStickyFault_Hardware().getValue())
            msg += "Hardware ";

        if(talon.getStickyFault_MissingDifferentialFX().getValue())
            msg += "MissingDifferentialFX ";
        if(talon.getStickyFault_OverSupplyV().getValue())
            msg += "OverSupplyV ";
        if(talon.getStickyFault_ProcTemp().getValue())
            msg += "ProcTemp ";
        if(talon.getStickyFault_RemoteSensorDataInvalid().getValue())
            msg += "RemoteSensorDataInvalid ";
        if(talon.getStickyFault_RemoteSensorPosOverflow().getValue())
            msg += "RemoteSensorPosOverflow ";
        if(talon.getStickyFault_RemoteSensorReset().getValue())
            msg += "RemoteSensorReset ";
        if(talon.getStickyFault_ReverseHardLimit().getValue())
            msg += "ReverseHardLimit ";

        if(talon.getStickyFault_ReverseSoftLimit().getValue())
            msg += "ReverseSoftLimit ";
        if(talon.getStickyFault_StatorCurrLimit().getValue())
            msg += "StatorCurrLimit ";
        if(talon.getStickyFault_SupplyCurrLimit().getValue())
            msg += "SupplyCurrLimit ";
        if(talon.getStickyFault_Undervoltage().getValue())
            msg += "Undervoltage ";
        if(talon.getStickyFault_UnlicensedFeatureInUse().getValue())
            msg += "UnlicensedFeatureInUse ";
        if(talon.getStickyFault_UnstableSupplyV().getValue())
            msg += "UnstableSupplyV ";
        if(talon.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue())
            msg += "UsingFusedCANcoderWhileUnlicensed ";
        
        if(msg == "") {
            msg = "Ok";
        }
        return msg;
    }

    private void readTalon(String name, BaseTalon talon) {
        String faultStr;
        String sFaultStr;
        //reading the raw bits because we know there are faults not in Faults (aka Neutral Brake Current Limit)
        var handle = talon.getHandle();
        var faultBits = MotControllerJNI.GetFaults(handle);
        var error = talon.getLastError();
        var sfaultBits = MotControllerJNI.GetStickyFaults(handle);
        var error2 = talon.getLastError();

        if (error != ErrorCode.OK) {
            faultStr = error.name();
        } else if (faultBits > 0) {
            faultStr = readTalonFaults(faultBits);
        } else {
            faultStr = "Ok";
        }

        if (error2 != ErrorCode.OK) {
            sFaultStr = error2.name();
        } else if (sfaultBits > 0) {
            sFaultStr = readTalonFaults(sfaultBits);
        } else {
            sFaultStr = "Ok";
        }

        commandTable.getEntry(name).setDouble(talon.getMotorOutputVoltage());
        currentTable.getEntry(name).setDouble(talon.getSupplyCurrent());
        faultTable.getEntry(name).setString(faultStr);
        stickyTable.getEntry(name).setString(sFaultStr);
        tempTable.getEntry(name).setDouble(talon.getTemperature());
        canStatusTable.getEntry(name).setString(talon.getLastError().name());
    }

    private String readTalonFaults(int bits) {
        Faults faults = new Faults();
        String retVal;

        faults.update(bits);
        retVal = readFaultStruct(faults);
        if(retVal.length() == 0) {
            retVal = "Unknown Fault " + bits;
        }

        return retVal;
    }

    private String readFaultStruct(Object obj) {
        StringBuilder work = new StringBuilder();

        //iterate through all the fields and find the boolean ones
        Field fieldlist[] = obj.getClass().getDeclaredFields();
        for (int i = 0; fieldlist.length > i; i++) {
            Field fld = fieldlist[i];
            if (fld.getType().equals(boolean.class)) {
                try {
                    boolean value = fld.getBoolean(obj);
                    if(value) {
                        work.append(fld.getName() + " ");
                    }
                } catch (IllegalArgumentException e) {
                    System.out.println("IllegalFaultReadArg");
                } catch (IllegalAccessException e) {
                    System.out.println("IllegalFaultReadAccess");
                }
            }
        }

        //check if string is empty
        if(work.length() == 0) {
            work.append("Ok");
        }
        return work.toString();
    }

    private String readSparkFaults(short faults) {
        if(faults == 0) {
            return "Ok";
        }
        StringBuilder work = new StringBuilder();
        for(var i=0; i<15; i++) {
            if((faults & (1 << i)) == (1 << i)) {
                var fault = CANSparkMax.FaultID.fromId(i);
                work.append(fault.name()).append(" ");
            }
        }
        return work.toString();
    }

    public static void checkClearFaults(boolean clear) {
        var clearFaults = SmartDashboard.getBoolean("Clear Faults", false);

        if(clearFaults == false && clear == false) {
            return;
        }
        SmartDashboard.putBoolean("Clear Faults", false);

        for (String name : items.keySet()) {
            var item = items.get(name);
            if(item instanceof BaseTalon) {
                var talon = (BaseTalon)item;
                talon.clearStickyFaults();
            } else if(item instanceof CANCoder) {
                var coder = (CANCoder)item;
                coder.clearStickyFaults();
            } else if(item instanceof CANSparkMax) {
                spark = (CANSparkMax)item;
                spark.clearFaults();
            } else {
                //unknown table
            }
        }

        if(pdp != null) {
            pdp.clearStickyFaults();
        }

        if(ph != null) {
            ph.clearStickyFaults();
        }

        if(pigeon != null) {
            pigeon.clearStickyFaults();
        }
    }

    public static boolean FaultSet() {
        return faultSet;
    }

    public static boolean StickyFaultSet() {
        return sfaultSet;
    }

    public static boolean IsFlashDriveAttached() {
        return flashDriveAttached;
    }
}
