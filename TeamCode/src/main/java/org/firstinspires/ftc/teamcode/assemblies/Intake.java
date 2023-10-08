package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.util.RobotLog;

public class Intake {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
}
