package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

public class Output {

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public static int elevatorMax = 2100; //Could maybe go to 2200 but playing it safe for now
    public static int elevatorMin = 0; //bottom
    public static int elevatorSafeFlipRotateLevel = 400;
    public static int elevatorSafeStrafeLevel = 700;

    public static int elevatorMaxVelocity = 2500;

    public static double GrabberRotatorLowerLimit = 0.427;
    public static double GrabberRotatorHorizontal1 = 0.515;
    public static double GrabberRotatorLoad= 0.568; // Vertical for loading
    public static double GrabberRotatorHorizontal2 = 0.6216; // 180 from Horizontal1
    public static double GrabberRotatorHorizontal3 = 0.7317; // This is the limit in one direction
    public static double GrabberRotatorUpperLimit = GrabberRotatorHorizontal3;
    public static double GrabberRotatorHorizontalToAngleIncrement = (GrabberRotatorHorizontal2 - GrabberRotatorHorizontal1) * 45/180;
    public static double GrabberRotatorAngleToAngleIncrement = (GrabberRotatorHorizontal2 - GrabberRotatorHorizontal1) * 90/180;


    public static double flipperLoad = 0.1516;
    public static double flipperScore = 0.845;

    public static double StraferLoad = 0.55;
    public static double StraferRight = 0.25;
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
    public Servo grabber;
    public Servo grabberRotater;
    public Servo grabberStrafer;
    public Servo flipper;

    public Servo liftArm;
    private DcMotorEx liftSpindle;

    private AtomicBoolean moving;

    public Output(){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize(){
        teamUtil.log("Initializing Output");
        elevLeft = hardwareMap.get(DcMotorEx.class, "elevLeft");
        elevRight = hardwareMap.get(DcMotorEx.class, "elevRight");
        elevLeft.setDirection(DcMotor.Direction.REVERSE); // Positive is UP on both motors
        moving = new AtomicBoolean(false);

        grabber = hardwareMap.get(Servo.class,"grabber");
        grabberRotater = hardwareMap.get(Servo.class,"grabberRotator");
        grabberStrafer = hardwareMap.get(Servo.class,"grabberStrafer");
        flipper = hardwareMap.get(Servo.class,"flipper");

        liftSpindle = hardwareMap.get(DcMotorEx.class, "liftSpindle");
        liftArm = hardwareMap.get(Servo.class,"liftArm");

        teamUtil.log("Output Initialized ");
    }

    public void calibrate(){ //reset and initialize arms
        log("Output Calibrate called");
        moving.set(true);
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
        elevRight.setVelocity(elevatorMaxVelocity); // TODO: Adjust this number to something appropriate
        elevLeft.setVelocity(elevatorMaxVelocity);
        moving.set(false);
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
        log("Elev Manual: " + increment);
        elevLeft.setTargetPosition((int)(clamp(elevLeft.getCurrentPosition()+increment,elevatorMin,elevatorMax)));
        elevRight.setTargetPosition((int)(clamp(elevRight.getCurrentPosition()+increment,elevatorMin,elevatorMax)));
   }

   public void straferManual(double increment){
        //TODO: This implementation is cursed.  getPosition on a Servo will return the last position it was commanded to, NOT the actual position.
        // TODO: So this will VERY quickly go to an extreme value.  ALSO, needs to be limited to the range limits.
        //grabberStrafer.setPosition(grabberStrafer.getPosition()+increment);
   }


    public void grabberRotatorManual(double increment){
        grabberRotater.setPosition(grabberRotater.getPosition()+increment);
    }

    public void moveLift(){
       liftSpindle.setVelocity(liftSpindleVelocity);
    }

    // SAFELY Return the output mechanisms to their position for loading pixels
    public void goToLoad() {
        moving.set(true);

        log("Go To Load");
        if (elevLeft.getCurrentPosition() < elevatorSafeStrafeLevel || elevRight.getCurrentPosition() < elevatorSafeStrafeLevel) {
            // we don't know where the servos are so we need to go up to a safe level to move them
            log("Go To Load: Raising to safe level");
            elevRight.setTargetPosition(elevatorSafeStrafeLevel);
            elevLeft.setTargetPosition(elevatorSafeStrafeLevel);
            while (elevLeft.getCurrentPosition() < elevatorSafeStrafeLevel-20 || elevRight.getCurrentPosition() < elevatorSafeStrafeLevel-20) {
            }
        }
        // Move servos to correct position and wait for them to complete
        // TODO: The timing on this can be optimized depending on the starting position of the lift
        log("Go To Load: Positioning Servos");
        grabberStrafer.setPosition(StraferLoad);
        grabberRotater.setPosition(GrabberRotatorLoad);
        flipper.setPosition(flipperLoad);
        grabber.setPosition(GrabberOpen);

        teamUtil.pause(1000); // Wait long enough for all servos to be in a safe position to go to load

        // Take elevator to the bottom
        log("Go To Load: Running to Bottom");
        elevRight.setTargetPosition(elevatorMin);
        elevLeft.setTargetPosition(elevatorMin); // TODO: Might want to turn motors off once we are at bottom to conserve power for driving, etc.

        log("Go To Load-Finished");
        moving.set(false);

    }

    public void goToLoadNoWait() {
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to goToLoad while output system is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to Stow Output System");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToLoad();
                }
            });
            thread.start();
        }
    }

    public void outputTelemetry() {
        telemetry.addData("Output  ", "ElevL: %d, ElevR: %d", elevLeft.getCurrentPosition(), elevRight.getCurrentPosition());
        telemetry.addData("Output  ", "Flip: %f, Strafe: %f, Rotate: %f, Grab: %f",
                flipper.getPosition(), grabberStrafer.getPosition(), grabberRotater.getPosition(), grabber.getPosition());
        telemetry.addData("Lift  ", "Motor: %d, Arm: %f", liftSpindle.getCurrentPosition(), liftArm.getPosition());
    }
}

