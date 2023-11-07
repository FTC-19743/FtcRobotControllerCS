package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Drive drive;
    public Intake intake;
    public Output output;
    public Lift lift;
    public Launcher launcher;


    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);

    }

    public Robot(){
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new Drive();
        intake = new Intake();
        output = new Output(intake);
        launcher = new Launcher();
        lift = new Lift();
    }

    public void initialize(){
        drive.initalize();
        intake.initalize();
        output.initialize();
        lift.initialize();
        launcher.initialize();

    }

    public void outputTelemetry(){
        drive.driveMotorTelemetry();
        output.outputTelemetry();
    }

    public void calibrate(){
        output.calibrate();
        output.goToLoad(); // ready to load
    }

    public int fieldSide() { // helper method that returns heading out towards the field
        return teamUtil.alliance == teamUtil.Alliance.RED ? 90 : 270;
    }
    public int driverSide() { // helper method that returns heading backs towards drivers
        return teamUtil.alliance == teamUtil.Alliance.RED ? 270 : 90;
    }

    public void auto(int path, boolean left){ // TODO: Lose the "left" parameter and use TeamUtil.SIDE instead
        drive.setHeading(180); // Zero is towards the scoring side of field
        if(path==1&&left){

        }
        else if(path==2&&left){

        }
        else if(path==3&&left){

        }
        else if(!left&&path==1){
            //out 32.5 inches
            drive.moveCm(82.55,fieldSide());
        }

        else if(!left&&path==2){
            drive.runRearAprilTagProcessor(); // Get AprilTag Finder up and running
            drive.moveCm(82.55,fieldSide());
            drive.moveCm(5,driverSide());
            drive.moveCm(60,0,350);//TODO:change to min end when callibrated (also, should be more like 80)
            teamUtil.pause(500); // TODO: Why?

            // TODO: There are many issues with this while loop.  It needs to hold the heading, have a timeout, and be responsive to someone
            // TODO: shutting down the op mode.  You need methods for operations like this.  study this example and emulate
            // TODO: In fact, all of your autonomous movement methods need timeouts and tests for opmode shut down
            drive.driveToTape(0,180,350,3000);
            //while (!drive.tapeSensor1.isOnTape()||!drive.tapeSensor2.isOnTape()){}

            drive.setMotorsActiveBrake();

        }
        teamUtil.pause(1000); // TODO: Remove this for competition, just helps with debugging auto as it is developed

    }

}


