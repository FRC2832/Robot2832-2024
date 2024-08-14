package org.livoniawarriors;

import java.lang.reflect.Field;
import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

public class UtilFunctions {
    /**
     * This function takes a joystick input from -1 to 1 and removes the center of the stick.
     * This is because Xbox joysticks have awful centering
     * @param input Joystick input to manipulate
     * @param deadband How much of the center we need to remove (Xbox 360 controllers was around 0.2, Xbox One 0.13)
     * @return A value between -1 to 1 that will not drift with stick drift
     */
    public static double deadband(double input, double deadband) {
        double abs = Math.abs(input);

        if (abs > deadband) {
            return Math.signum(input) * ((abs-deadband)/(1-deadband));
        } else {
            return 0;
        }
    }

    /**
     * This function takes a input in degrees and makes it -180 to 180*.
     * If you are in radians, use MathUtil.angleModulus() from WpiLib
     * @param degAngle Angle to reduce
     * @return A value between -180 to 180*
     */
    public static double degreeMod(double degAngle) {
        return MathUtil.inputModulus(degAngle,-180,180);
    }

    /**
     * This uses the Preferences API to save settings over power cycles.
     * This is different in that you don't have to set the default value, it will set it for you.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static double getSetting(String key, double backup) {
        if(Preferences.containsKey(key)) {
            //key exists, return the value
            return Preferences.getDouble(key, backup);
        } else {
            //key missing, set default
            Preferences.initDouble(key, backup);
            return backup;
        }
    }

    /**
     * This uses the Preferences API to save settings over power cycles.
     * This is different in that you don't have to set the default value, it will set it for you.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The value in NetworkTables if it exists, the backup if missing
     */
    public static boolean getSetting(String key, boolean backup) {
        if(Preferences.containsKey(key)) {
            //key exists, return the value
            return Preferences.getBoolean(key, backup);
        } else {
            //key missing, set default
            Preferences.initBoolean(key, backup);
            return backup;
        }
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The subscriber to get values from
     */
    public static DoubleSubscriber getSettingSub(String key, double backup) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(checkKey("/Preferences/" + key));
        DoublePublisher pub = topic.publish();
        pub.setDefault(backup);
        DoubleSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        topic.setPersistent(true);
        return sub;
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * It will locate the key in the Preferences table still.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The subscriber to get values from
     */
    public static BooleanSubscriber getSettingSub(String key, boolean backup) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(checkKey("/Preferences/" + key));
        BooleanPublisher pub = topic.publish();
        pub.setDefault(backup);
        BooleanSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        topic.setPersistent(true);
        return sub;
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The subscriber to get values from
     */
    public static BooleanSubscriber getNtSub(String key, boolean backup) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(checkKey(key));
        BooleanPublisher pub = topic.publish();
        pub.setDefault(backup);
        BooleanSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        return sub;
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The subscriber to get values from
     */
    public static DoubleArraySubscriber getNtSub(String key, double[] backup) {
        DoubleArrayTopic topic = NetworkTableInstance.getDefault().getDoubleArrayTopic(checkKey(key));
        DoubleArrayPublisher pub = topic.publish();
        pub.setDefault(backup);
        DoubleArraySubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        return sub;
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The subscriber to get values from
     */
    public static IntegerSubscriber getNtSub(String key, int backup) {
        IntegerTopic topic = NetworkTableInstance.getDefault().getIntegerTopic(checkKey(key));
        IntegerPublisher pub = topic.publish();
        pub.setDefault(backup);
        IntegerSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        return sub;
    }

    /**
     * This creates a NT subscriber so we don't have to keep querying the key in the table to get the value.
     * @param key The parameter you want to get (slashes are allowed)
     * @param backup The value to use if the key is missing
     * @return The subscriber to get values from
     */
    public static DoubleSubscriber getNtSub(String key, double backup) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(checkKey(key));
        DoublePublisher pub = topic.publish();
        pub.setDefault(backup);
        DoubleSubscriber sub = topic.subscribe(backup);
        if(!sub.exists()) {
            pub.set(backup);
        }
        return sub;
    }

    /**
     * This creates a NT publisher so we don't have to keep querying the key in the table.
     * @param key The parameter you want to get (slashes are allowed)
     * @return The publisher to put data in
     */
    public static BooleanPublisher getNtPub(String key, boolean initValue) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(checkKey(key));
        BooleanPublisher pub = topic.publish();
        pub.setDefault(initValue);
        return pub;
    }

    /**
     * This creates a NT publisher so we don't have to keep querying the key in the table.
     * @param key The parameter you want to get (slashes are allowed)
     * @return The publisher to put data in
     */
    public static StringPublisher getNtPub(String key, String initValue) {
        StringTopic topic = NetworkTableInstance.getDefault().getStringTopic(checkKey(key));
        StringPublisher pub = topic.publish();
        pub.setDefault(initValue);
        return pub;
    }

    /**
     * This creates a NT publisher so we don't have to keep querying the key in the table.
     * @param key The parameter you want to get (slashes are allowed)
     * @return The publisher to put data in
     */
    public static DoublePublisher getNtPub(String key, double initValue) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(checkKey(key));
        DoublePublisher pub = topic.publish();
        pub.setDefault(initValue);
        return pub;
    }

    /**
     * This creates a NT publisher so we don't have to keep querying the key in the table.
     * @param key The parameter you want to get (slashes are allowed)
     * @return The publisher to put data in
     */
    public static IntegerPublisher getNtPub(String key, long initValue) {
        IntegerTopic topic = NetworkTableInstance.getDefault().getIntegerTopic(checkKey(key));
        IntegerPublisher pub = topic.publish();
        pub.setDefault(initValue);
        return pub;
    }

    /**
     * This creates a NT publisher so we don't have to keep querying the key in the table.
     * @param key The parameter you want to get (slashes are allowed)
     * @return The publisher to put data in
     */
    public static DoubleArrayPublisher getNtPub(String key, double[] initValue) {
        DoubleArrayTopic topic = NetworkTableInstance.getDefault().getDoubleArrayTopic(checkKey(key));
        DoubleArrayPublisher pub = topic.publish();
        pub.setDefault(initValue);
        return pub;
    }

    public static BooleanEntry getNtEntry(String key, boolean initValue) {
        BooleanTopic topic = NetworkTableInstance.getDefault().getBooleanTopic(checkKey(key));
        BooleanEntry entry = topic.getEntry(initValue);
        entry.setDefault(initValue);
        return entry;
    }

    /**
     * Add a event listener for when a network table value changes remotely.
     * @param key What network table key to monitor
     * @param listener Function to run when the value changes.  Could be a function taking a NetworkTableEvent
     * parameter or a lambda function.
     * @return Handle that can be used in NetworkTableInstance.getDefault().removeListener(x)
     */
    public static int onNtChange(String key, Consumer<NetworkTableEvent> listener) {
        DoubleTopic topic = NetworkTableInstance.getDefault().getDoubleTopic(checkKey("/Preferences/" + key));
        DoubleSubscriber sub = topic.subscribe(0);

        // add a listener to only value changes on the Y subscriber
        return NetworkTableInstance.getDefault().addListener(
            sub,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            listener);
    }

    private static String checkKey(String key) {
        String newKey;

        if (key.startsWith("/", 0)) {
            newKey = key;
        } else {
            newKey = "/" + key;
        }

        newKey = newKey.replaceAll("//", "/");
        return newKey;
    }

    /**
     * This function adds a periodic function to the schedule.  This will run after the main loop finishes.
     * @param callback Function to run
     * @param periodSeconds How often to run the function in seconds
     * @param offsetSeconds What offset to run this function at
     * @return
     */
    public static boolean addPeriodic(Runnable callback, double periodSeconds, double offsetSeconds) {
        try {
            Field field = RobotBase.class.getDeclaredField("m_robotCopy");
            field.setAccessible(true);
            TimedRobot returnObject = (TimedRobot)field.get(RobotBase.class);
            returnObject.addPeriodic(callback, periodSeconds, offsetSeconds);
            return true;
        } catch (Exception e) {
            //don't do anything, we just return false that it didn't schedule
        } 
        return false;
    }

    public static double getDistance(Pose2d pose1, Pose2d pose2) {
        double xDist = pose1.getX() - pose2.getX();
        double yDist = pose1.getY() - pose2.getY();
        return Math.sqrt((xDist * xDist) + (yDist * yDist));
    }

    public static double getDistance(Pose3d pose1, Pose3d pose2) {
        double xDist = pose1.getX() - pose2.getX();
        double yDist = pose1.getY() - pose2.getY();
        double zDist = pose1.getZ() - pose2.getZ();
        return Math.sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
    }

    public static double getDistance(Transform3d pose) {
        double xDist = pose.getX();
        double yDist = pose.getY();
        double zDist = pose.getZ();
        return Math.sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
    }

    public static double getAngle(Pose2d pose1, Pose2d pose2) {
        double xDist = pose1.getX() - pose2.getX();
        double yDist = pose1.getY() - pose2.getY();
        double angle = Math.atan(yDist/xDist);
        return angle;
    }
    
    public static double LimitChange(double current, double target, double maxChangePerLoop) {
        double delta = target - current;
        double newValue;

        if(Math.abs(delta) > maxChangePerLoop) {  
            newValue = current + Math.copySign(maxChangePerLoop, delta);
        } else {
            newValue = target;
        }
        return newValue;
    }

    /* TODO: Move this for next season, in VisionSystem for now */
    private static Alliance alliance;
    public static Alliance getAlliance() {
        return alliance;
    }

    public static void setAlliance(Alliance alliance) {
        UtilFunctions.alliance = alliance;
    }
}
