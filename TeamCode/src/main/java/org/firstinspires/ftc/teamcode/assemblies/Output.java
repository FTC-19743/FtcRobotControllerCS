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

    public boolean loading = false;
    public static int elevatorMax = 2100; //Could maybe go to 2200 but playing it safe for now
    public static int elevatorMin = 0; //bottom
    public static int elevatorSafeFlipRotateLevel = 400;
    public static int elevatorSafeStrafeLevel = 700;
    public static int elevatorMinScoreLevel = 275; // Lowest level for scoring horizontally
    public static int elevatorScoreLevel3 = 690; // Lowest level for scoring horizontally
    public static int elevatorScoreInc = elevatorScoreLevel3 - elevatorMinScoreLevel / 2; // amount to lower or raise elevator per level
    public static int elevatorRotateInc = elevatorScoreInc / 2; // amount to lower or raise elevator when changing from horizontal to slanted placement

    public static int elevatorMaxVelocity = 2500; //TODO: Could be a bit higher?

    public static double GrabberRotatorLowerLimit = 0.427;
    public static double GrabberRotatorHorizontal1 = 0.515;
    public static double GrabberRotatorLoad= 0.568; // Vertical for loading
    public static double GrabberRotatorHorizontal2 = 0.6216; // 180 from Horizontal1
    public static double GrabberRotatorHorizontal3 = 0.7317; // This is the limit in one direction
    public static double GrabberRotatorUpperLimit = GrabberRotatorHorizontal3;
    public static double GrabberRotatorIncrement = (GrabberRotatorHorizontal2 - GrabberRotatorHorizontal1) * 60/180;



    public static double flipperLoad = 0.1516;
    public static double flipperScore = 0.845;

    public static double StraferLoad = 0.55;
    public static double StraferRight = 0.25;
    public static double StraferLeft = 0.86;

    public static double StraferIncrement = 0.025;
    public static double GrabberOpen = 0.55;
    public static double GrabberClosed = 0.73;
    public static double GrabberOnePixel = .81;
    public static int StallBuffer = 225;

    public DcMotorEx elevLeft;
    public DcMotorEx elevRight;
    public Servo grabber;
    public Servo grabberRotater;
    public int lastLevel = 2;

    public enum rotationPosition {
        HORIZONTAL, PIOVERTHREE, TWOPIOVERTHREE, VERTICAL
    }
    public rotationPosition rotaterPosition;
    public Servo grabberStrafer;
    public Servo flipper;

    public Intake intake;


    private AtomicBoolean moving;

    public Output(Intake i){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        intake = i;

    }

    public void initialize(){
        teamUtil.log("Initializing Output");
        elevLeft = hardwareMap.get(DcMotorEx.class, "elevLeft");
        elevRight = hardwareMap.get(DcMotorEx.class, "elevRight");
        elevLeft.setDirection(DcMotor.Direction.REVERSE); // Positive is UP on both motors
        moving = new AtomicBoolean(false);

        flipper = hardwareMap.get(Servo.class,"flipper");
        flipper.setPosition(flipperLoad);
        grabber = hardwareMap.get(Servo.class,"grabber");
        grabberRotater = hardwareMap.get(Servo.class,"grabberRotator");
        grabberStrafer = hardwareMap.get(Servo.class,"grabberStrafer");

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
        elevRight.setVelocity(elevatorMaxVelocity);
        elevLeft.setVelocity(elevatorMaxVelocity);
        moving.set(false);
        log("Output Calibrate finished");
    }

    public void dropPixels(){
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to drop pixels while output system is moving--ignored");
            return;
        } else {
            grabber.setPosition(GrabberOpen);
        }
    }

    public void grabPixels(){
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to grab pixels while output system is moving--ignored");
            return;
        } else {
            grabber.setPosition(GrabberClosed);
        }
    }

    public void grabOnePixel(){
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to grab pixels while output system is moving--ignored");
            return;
        } else {
            grabber.setPosition(GrabberOnePixel);
        }
    }

   public void rotateGrabberStart(){

   }

   public void rotateGrabberClockwise(){
        if(loading == false) {
            if (grabberRotater.getPosition() + GrabberRotatorIncrement <= GrabberRotatorUpperLimit) {
                grabberRotater.setPosition(grabberRotater.getPosition() - GrabberRotatorIncrement);
            }
        }
   }

    public void rotateGrabberCounterclockwise(){
        if(loading == false) {
            if (grabberRotater.getPosition() - GrabberRotatorIncrement >= GrabberRotatorLowerLimit) {
                grabberRotater.setPosition(grabberRotater.getPosition() + GrabberRotatorIncrement);
            }
        }
    }



   public void elevManual(double increment){
       if (moving.get() || loading) { // Output system is already moving in a long running operation
           teamUtil.log("WARNING: Attempt to move elevator while output system is moving--ignored");
           return;
       } else {
           log("Elev Manual: " + increment);
           elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() + increment, elevatorMin, elevatorMax)));
           elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() + increment, elevatorMin, elevatorMax)));
       }
   }
//    public void elevatorControl(joystick){
//        if (moving.get() || loading) { // Output system is already moving in a long running operation
//            teamUtil.log("WARNING: Attempt to move elevator while output system is moving--ignored");
//            return;
//        } else {
//            log("Elev Manual: " + increment);
//            elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() + increment, elevatorMin, elevatorMax)));
//            elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() + increment, elevatorMin, elevatorMax)));
//        }
//    }
   public void straferManual(boolean left){

       if (moving.get() || loading) { // Output system is already moving in a long running operation
           teamUtil.log("WARNING: Attempt to strafe grabber while output system is moving--ignored");
           return;
       } else {
           if(left){
               if(grabberStrafer.getPosition()+StraferIncrement>StraferLeft){
                   grabberStrafer.setPosition(StraferLeft);
               }
               else{
                   grabberStrafer.setPosition(grabberStrafer.getPosition()+StraferIncrement);

               }

           }else{
               if(grabberStrafer.getPosition()-StraferIncrement<StraferRight){
                   grabberStrafer.setPosition(StraferRight);
               }
               else{
                   grabberStrafer.setPosition(grabberStrafer.getPosition()-StraferIncrement);

               }
           }
       }
   }

    public void setServosToLoad() {
        grabberStrafer.setPosition(StraferLoad);
        grabberRotater.setPosition(GrabberRotatorLoad);
        rotaterPosition = rotaterPosition.VERTICAL;
        flipper.setPosition(flipperLoad);
        grabber.setPosition(GrabberOpen);
        teamUtil.pause(1000); // Wait long enough for all servos to be in a safe position to go to load
        // TODO LATER: The timing on this can be optimized depending on the starting position of the lift
    }

    public void goToLoadNoElevator() {
        moving.set(true);
        log("Go To Load No Elevator");
         // Move servos to correct position and wait for them to complete
        setServosToLoad();
        log("Go To Load No Elevator-Finished");
        moving.set(false);
        loading = true;
    }

    // SAFELY Return the output mechanisms to their position for loading pixels
    public void goToLoad() {
        lastLevel = (elevLeft.getCurrentPosition()-elevatorMinScoreLevel)/elevatorScoreInc-1;
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
        log("Go To Load: Positioning Servos");
        setServosToLoad();

        // Take elevator to the bottom
        log("Go To Load: Running to Bottom");
        elevRight.setTargetPosition(elevatorMin);
        elevLeft.setTargetPosition(elevatorMin); // TODO LATER: Might want to turn motors off once we are at bottom to conserve power for driving, etc.

        while (elevLeft.getCurrentPosition() > elevatorMin+10 || elevRight.getCurrentPosition() > elevatorMin+10) {
        }
        elevLeft.setVelocity(0);
        elevRight.setVelocity(0);
        log("Go To Load-Finished");
        moving.set(false);
        loading = true;

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
    // Grab the pixels and get into scoring position
    public void goToScore(int level) {
        moving.set(true);
        int elevDestination = elevatorMinScoreLevel + (level-1)*elevatorScoreInc;
        elevLeft.setVelocity(elevatorMaxVelocity);
        elevRight.setVelocity(elevatorMaxVelocity);
        log("Go To Score");

        //if (intake.twoPixelsPresent() == true){
            if (grabber.getPosition() < GrabberOpen + .1) { // grabber is currently open
                grabPixels();
                teamUtil.pause(250);
            }
        //}
        //else{
            if(grabber.getPosition()< GrabberOpen + .1) {
                grabOnePixel();
                teamUtil.pause(250);
            }
            //In theory this would grab either one or two pixels depending on whether or not twoPixelsPresent() is true
        //}

        intake.stopIntake();

        elevLeft.setTargetPosition(Math.max(elevDestination,elevatorScoreLevel3));
        elevRight.setTargetPosition(Math.max(elevDestination,elevatorScoreLevel3));


        while (elevLeft.getCurrentPosition() < elevatorSafeFlipRotateLevel || elevRight.getCurrentPosition() < elevatorSafeFlipRotateLevel) {
        }
        flipper.setPosition(flipperScore);
        grabberRotater.setPosition(GrabberRotatorHorizontal2);
        rotaterPosition = rotaterPosition.HORIZONTAL;
        elevLeft.setTargetPosition(elevDestination);
        elevRight.setTargetPosition(elevDestination);

        while(Math.abs(elevLeft.getCurrentPosition()-elevDestination)>100||Math.abs(elevRight.getCurrentPosition()-elevDestination)>100){
        }

        log("Go To Score-Finished");
        moving.set(false);
        loading = false;
    }


    public void goToScoreNoWait(int level) {
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to goToLoad while output system is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to Stow Output System");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToScore(level);
                }
            });
            thread.start();
        }
    }





    public void outputTelemetry() {
        telemetry.addData("Output  ", "ElevL: %d, ElevR: %d", elevLeft.getCurrentPosition(), elevRight.getCurrentPosition());
        telemetry.addData("Output  ", "Flip: %f, Strafe: %f, Rotate: %f, Grab: %f",
                flipper.getPosition(), grabberStrafer.getPosition(), grabberRotater.getPosition(), grabber.getPosition());
    }
}

