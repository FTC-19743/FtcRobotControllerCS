package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Output {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public static int PulleyTelescopeRetracted = 0; //tentative value

    public static int PulleyTelescopeExtended = 1000; //tentative value

    public static double GrabberRotatorStart = 0; //tentative value

    public static double GrabberRotatorFlipped= 1.0; //tentative value

    public static double GrabberOpen = 1.0; //tentative value

    public static double GrabberClosed = 0; //tentative value


    private DcMotorEx liftLeft;

    private DcMotorEx liftRight;

    public Servo grabber;

    public Servo grabberRotater;

    public Servo grabberStrafer;

    public Servo flipper;


    Telemetry telemetry;
    HardwareMap hardwareMap;


    public Output(){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        Telemetry telemetry = teamUtil.theOpMode.telemetry;

    }

    public void initialize(){
        teamUtil.log("Initializing Intake");

        liftLeft = hardwareMap.get(DcMotorEx.class, "lift_left");
        liftRight = hardwareMap.get(DcMotorEx.class, "lift_right");
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE); //tentative (true direction unknown)

        grabber = hardwareMap.get(Servo.class,"direct_output_grabber");
        grabberRotater = hardwareMap.get(Servo.class,"grabber_rotator");
        grabberStrafer = hardwareMap.get(Servo.class,"grabber_strafer");
        flipper = hardwareMap.get(Servo.class,"flipper");

    }

    public void resetLift(){ //reset and initialize arms method

    }

    public void dropPixels(){
        grabber.setPosition(GrabberOpen);
    }

    public void grabPixels(){
        grabber.setPosition(GrabberClosed);

    }

   public void rotateGrabberStart(){

   }


}

