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
        output.initalize();
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

    public void auto(int path, boolean left){
        drive.setHeading(180);
        output.grabOnePixel();
        if(path==1&&left){

        }
        else if(path==2&&left){

        }
        else if(path==3&&left){

        }
        else if(!left&&path==1){
            //out 32.5 inches
            drive.moveCm(82.55,270);
        }

        else if(!left&&path==2){
            //out 32.5 inches
            drive.moveCm(82.55,90);
            drive.moveCm(5,270);
            drive.moveCm(60,0,350);//TODO:change to min end when callibrated
            teamUtil.pause(500);
            while (!drive.tapeSensor1.isOnTape()){ // TODO:change this to either
            }
            drive.setMotorsActiveBrake();

        }

    }

}


