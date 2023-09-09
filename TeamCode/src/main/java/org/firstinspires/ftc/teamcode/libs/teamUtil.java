package org.firstinspires.ftc.teamcode.libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teamUtil {

    public static LinearOpMode theOpMode;
    public static boolean inInitialization = true;
    public static Telemetry telemetry;
    public static boolean LEFT;
    public enum Alliance {RED, BLUE}

    public static Alliance alliance = Alliance.RED;

    public static void init (LinearOpMode opMode) {
        theOpMode = opMode;
        telemetry = theOpMode.telemetry;
        inInitialization = true;
    }

    // Wait for the specified milliseconds
    public static void pause(long sleepTime) {
        long wakeupTime= System.currentTimeMillis()+sleepTime;
        while(System.currentTimeMillis()< wakeupTime){
            theOpMode.idle();
        }
    }

    // log something so we can filter out the FTC robot log info
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    // A helper method to see if a long running operation should continue.
    // Call this in your loops
    public static boolean keepGoing(long timeOutTime) {
        String s=Long.toString(System.currentTimeMillis());

        //log(s);
        return ((inInitialization || theOpMode.opModeIsActive()) && (System.currentTimeMillis() < timeOutTime));
    }
}
