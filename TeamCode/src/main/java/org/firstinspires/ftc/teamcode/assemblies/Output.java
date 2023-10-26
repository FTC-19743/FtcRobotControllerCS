package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static double liftArmUp = 0.5; //tenative value
    public static double liftArmStowed = 0.0; // tenative value
    public static double liftSpindleVelocity = 1500; // tenative value
    public static int StallBuffer = 225;

     // TODO extension for lift makes encoder go negative



    private DcMotorEx elevLeft;

    private DcMotorEx elevRight;

    private DcMotorEx liftSpindle;
    public Servo grabber;

    public Servo grabberRotater;

    public Servo grabberStrafer;

    public Servo flipper;
    public Servo liftArm;

    Telemetry telemetry;
    HardwareMap hardwareMap;


    public Output(){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;

    }

    public void initalize(){
        teamUtil.log("Initializing Intake");



        elevLeft = hardwareMap.get(DcMotorEx.class, "elevLeft");
        elevRight = hardwareMap.get(DcMotorEx.class, "elevRight");
        elevRight.setDirection(DcMotor.Direction.REVERSE);
        elevLeft.setDirection(DcMotor.Direction.REVERSE);

        liftSpindle = hardwareMap.get(DcMotorEx.class, "liftSpindle");

        liftArm = hardwareMap.get(Servo.class,"liftArm");
        grabber = hardwareMap.get(Servo.class,"grabber");
        grabberRotater = hardwareMap.get(Servo.class,"grabberRotator");
        grabberStrafer = hardwareMap.get(Servo.class,"grabberStrafer");
        flipper = hardwareMap.get(Servo.class,"flipper");
//        grabberStrafer.setPosition(0.5);
//        grabberRotater.setPosition(0.5);
    }

    public void calibrate(){ //reset and initialize arms
        log("Output Calibrate called");
        elevRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        log("Calibrate: Running down slowly");
        elevRight.setPower(-.1);
        elevLeft.setPower(-.1);
        int lastLeftPosition = elevLeft.getCurrentPosition();
        int lastRightPosition = elevRight.getCurrentPosition();
        teamUtil.pause(250);
        while(elevLeft.getCurrentPosition()!=lastLeftPosition || elevRight.getCurrentPosition()!=lastRightPosition){
            lastLeftPosition = elevLeft.getCurrentPosition();
            lastRightPosition = elevRight.getCurrentPosition();
            log("Calibrate: Left: "+elevLeft.getCurrentPosition() + " Right: "+ elevRight.getCurrentPosition());

            teamUtil.pause(50);
        }
        elevRight.setPower(0);
        elevLeft.setPower(0);
        log("Calibrate: Stopped Motors");
        elevLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setTargetPosition(elevRight.getCurrentPosition());
        elevLeft.setTargetPosition(elevLeft.getCurrentPosition());
        log("Calibrate Final: Left: "+elevLeft.getCurrentPosition() + " Right: "+ elevRight.getCurrentPosition());
        elevLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevRight.setVelocity(300); // TODO: Adjust this number to something appropriate
        elevLeft.setVelocity(300);
        log("Output Calibrate finished");
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

   public void elevManual(double increment){
        elevLeft.setTargetPosition((int)(elevLeft.getCurrentPosition()+increment));
        elevRight.setTargetPosition((int)(elevRight.getCurrentPosition()+increment));

   }

   public void straferManual(double increment){
        grabberStrafer.setPosition(grabberStrafer.getPosition()+increment);
   }


    public void grabberRotatorManual(double increment){
        grabberRotater.setPosition(grabberRotater.getPosition()+increment);
    }

    public void moveLift(){
       liftSpindle.setVelocity(liftSpindleVelocity);
    }



    public void outputTelemetry() {
        telemetry.addData("Output  ", "elevLeft: %d, elevRight: %d, lift: %d",
                elevLeft.getCurrentPosition(), elevRight.getCurrentPosition(), liftSpindle.getCurrentPosition());


    }




}

