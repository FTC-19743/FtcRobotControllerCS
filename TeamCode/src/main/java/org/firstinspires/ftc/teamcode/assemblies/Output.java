package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
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

    public static double GrabberRotatorHorizontal = 0.29;

    public static double GrabberRotatorVertical= 0.345; //tentative value

    public static double StraferRotatorDown = 0.155;

    public static double StraferRotatorScoring = 0.85;

    public static double StraferRight = 0.25;

    public static double StraferLoad = 0.53;

    public static double StraferLeft = 0.86;


    public static double GrabberOpen = 0.55;

    public static double GrabberClosed = 0.73;

     // TODO extension for lift makes encoder go negative



    private DcMotorEx elevLeft;

    private DcMotorEx elevRight;

    private DcMotorEx lift;

    public Servo grabber;

    public Servo grabberRotater;

    public Servo grabberStrafer;

    public Servo flipper;


    Telemetry telemetry;
    HardwareMap hardwareMap;


    public Output(){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;

    }

    public void initalize(){
        teamUtil.log("Initializing Intake");



        elevLeft = hardwareMap.get(DcMotorEx.class, "elev_left");
        elevRight = hardwareMap.get(DcMotorEx.class, "elev_right");
        elevRight.setDirection(DcMotor.Direction.REVERSE);
        elevLeft.setDirection(DcMotor.Direction.REVERSE);

        //elevLeft.setDirection(DcMotorSimple.Direction.REVERSE); //tentative (true direction unknown)
        lift = hardwareMap.get(DcMotorEx.class, "lift");


        grabber = hardwareMap.get(Servo.class,"grabber");
        grabberRotater = hardwareMap.get(Servo.class,"grabberRotator");
        grabberStrafer = hardwareMap.get(Servo.class,"grabberStrafer");
        flipper = hardwareMap.get(Servo.class,"flipper");
        grabberStrafer.setPosition(0.5);
        grabberRotater.setPosition(0.5);




    }

    public void calibrate(){ //reset and initialize arms method
        elevRight.setVelocity(-100);
        elevLeft.setVelocity(-100);
    }

    public void dropPixels(){
        grabber.setPosition(GrabberOpen);
    }

    public void grabPixels(){
        grabber.setPosition(GrabberClosed);

    }

   public void rotateGrabberStart(){

   }

   public void goToLevel(int level){

   }

   public void elevManualIncrement(int increment){
        elevLeft.setTargetPosition(elevLeft.getCurrentPosition()+increment);
        elevRight.setTargetPosition(elevRight.getCurrentPosition()+increment);

   }

   public void straferAddManualIncrement(double increment){
        grabberStrafer.setPosition(grabberStrafer.getPosition()+increment);
   }



    public void straferSubtractManualIncrement(double increment){
        grabberStrafer.setPosition(grabberStrafer.getPosition()-increment);
    }

    public void grabberRotatorAddManualIncrement(double increment){
        grabberRotater.setPosition(grabberRotater.getPosition()+increment);
    }

    public void grabberRotatorSubtractManualIncrement(double increment){
        grabberRotater.setPosition(grabberRotater.getPosition()-increment);
    }

    public void manualLiftChange(double velocity){
       lift.setVelocity(velocity);
    }



    public void outputTelemetry() {
        telemetry.addData("Output  ", "elevLeft: %d, elevRight: %d, lift: %d",
                elevLeft.getCurrentPosition(), elevRight.getCurrentPosition(), lift.getCurrentPosition());


    }




}

