package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

public class Output {

    Telemetry telemetry;
    HardwareMap hardwareMap;

    public AtomicBoolean loading = new AtomicBoolean(false);
    public static int elevatorMax = 2100; //Could maybe go to 2200 but playing it safe for now
    public static int elevatorMin = 0; //bottom
    public static int elevatorSafeFlipRotateLevel = 400;
    public static int elevatorSafeStrafeLevel = 700;
    public static int elevatorMinScoreLevel = 275; // Lowest level for scoring horizontally
    public static int elevatorScoreLevel3 = 690; // Lowest level for scoring horizontally
    public static int elevatorScoreInc = (elevatorScoreLevel3 - elevatorMinScoreLevel) / 2; // amount to lower or raise elevator per level = 207.5
    public static int elevatorRotateInc = elevatorScoreInc / 2; // amount to lower or raise elevator when changing from horizontal to slanted placement

    public static int elevatorMaxVelocity = 2500; //TODO: Could be a bit higher?

    public static double GrabberRotatorLowerLimit = 0.427;
    public static double GrabberRotatorHorizontal1 = 0.515;
    public static double GrabberRotatorLoad= 0.568; // Vertical for loading
    public static double GrabberRotatorHorizontal2 = 0.6216; // 180 from Horizontal1
    public static double GrabberRotatorHorizontal3 = 0.7317; // This is the limit in one direction
    public static double GrabberRotatorUpperLimit = GrabberRotatorHorizontal3;
    public static double GrabberRotatorIncrement = (GrabberRotatorHorizontal2 - GrabberRotatorHorizontal1) * 60/180;



    //public static double flipperLoad = 0.1516+0.007*5;
    public static double flipperLoad = 0.5; // was 0.1877

    //public static double flipperScore = 0.845+0.007*5;
    public static double flipperScore = 0.85; // was .8756

    public static double StraferLoad = 0.5; // was 0.485
    public static double StraferRight = 0.297 ; // was .28  "Right" when facing the backdrop.  Actually Robot's left
    public static double StraferLeft = 0.675; // was .655

    public static double StraferPositionPerCm = .02364;

    public static int lastStraferPosition; //1=left,2=load,3=right

    public static double StraferIncrement = 0.015;

    public static long lastStraferServoSkew = 0;

    public static long servoSkewTimeMs = 75; //tentative value
    public static double GrabberOpen = 0.53;
    public static double GrabberClosed = 0.75;
    public static double GrabberOnePixel = .81;
    public static int StallBuffer = 225;
    public static double ElevCrawlIncrement = 30;
    public static double ElevFastIncrement = 100;

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
    boolean details = true;


    public AtomicBoolean moving;

    public Output(Intake i){
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        intake = i;

    }

    public void initialize(){
        teamUtil.log("Initializing Output");
        initializeElevs();
        initializeServos();

        teamUtil.log("Output Initialized ");
    }

    public void reset(){
        initializeElevs();
        elevLeft.setVelocity(elevatorMaxVelocity);
        elevRight.setVelocity(elevatorMaxVelocity);
        elevLeft.setTargetPosition(elevatorSafeFlipRotateLevel);
        elevRight.setTargetPosition(elevatorSafeFlipRotateLevel);
        elevLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        teamUtil.pause(1000);
        initializeServos();
        teamUtil.pause(1500);
    }

    public void initializeServos(){
        flipper = hardwareMap.get(Servo.class,"flipper");
        flipper.setPosition(flipperLoad);
        grabber = hardwareMap.get(Servo.class,"grabber");
        grabber.setPosition(GrabberOpen);
        grabberRotater = hardwareMap.get(Servo.class,"grabberRotator");
        grabberRotater.setPosition(GrabberRotatorLoad);
        grabberStrafer = hardwareMap.get(Servo.class,"grabberStrafer");
        grabberStrafer.setPosition(StraferLoad);
    }

    public void initializeElevs(){
        moving = new AtomicBoolean(false);
        loading.set(true);

        elevLeft = hardwareMap.get(DcMotorEx.class, "elevLeft");
        elevRight = hardwareMap.get(DcMotorEx.class, "elevRight");
        elevLeft.setDirection(DcMotor.Direction.REVERSE); // Positive is UP on both motors
        elevLeft.setVelocity(0);
        elevRight.setVelocity(0);
        elevLeft.setTargetPosition(elevLeft.getCurrentPosition());
        elevRight.setTargetPosition(elevRight.getCurrentPosition());
        elevLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void calibrate(){ //reset and initialize arms
        teamUtil.log("Output Calibrate called");
        moving.set(true);
        elevRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        teamUtil.log("Calibrate: Running down slowly");
        elevRight.setPower(-.1);
        elevLeft.setPower(-.1);
        int lastLeftPosition = elevLeft.getCurrentPosition();
        int lastRightPosition = elevRight.getCurrentPosition();
        teamUtil.pause(250);
        while(elevLeft.getCurrentPosition()!=lastLeftPosition || elevRight.getCurrentPosition()!=lastRightPosition){
            lastLeftPosition = elevLeft.getCurrentPosition();
            lastRightPosition = elevRight.getCurrentPosition();
            teamUtil.log("Calibrate: Left: "+elevLeft.getCurrentPosition() + " Right: "+ elevRight.getCurrentPosition());

            teamUtil.pause(50);
        }
        elevRight.setPower(0);
        elevLeft.setPower(0);
        teamUtil.log("Calibrate: Stopped Motors");
        elevLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setTargetPosition(elevRight.getCurrentPosition());
        elevLeft.setTargetPosition(elevLeft.getCurrentPosition());
        teamUtil.log("Calibrate Final: Left: "+elevLeft.getCurrentPosition() + " Right: "+ elevRight.getCurrentPosition());
        elevLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ///elevRight.setVelocity(elevatorMaxVelocity);
        ///elevLeft.setVelocity(elevatorMaxVelocity);
        moving.set(false);

        teamUtil.log("Output Calibrate finished");
    }



    public void dropPixels(){
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to drop pixels while output system is moving--ignored");
            return;
        } else {
            grabber.setPosition(GrabberOpen);

        }
    }
    public void dropAndGoToLoad() {
        teamUtil.log("Drop and Stow");
        grabber.setPosition(GrabberOpen);
        teamUtil.pause(1000);
        goToLoad();
    }

    public void dropAndGoToLoadNoWait() {
        if (moving.get()||loading.get()) { // Output system is already moving in a long running operation
            // log("states in go to load");
            /*log("moving" + moving.get());
            log("loading" + loading.get());*/
            teamUtil.log("WARNING: Attempt to goToLoad while output system is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to Stow Output System");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    dropAndGoToLoad();
                }
            });
            thread.start();
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
        if(!loading.get()) {
            if(Math.abs(grabberRotater.getPosition()-GrabberRotatorLoad)<.001){
                grabberRotater.setPosition(grabberRotater.getPosition() - GrabberRotatorIncrement/2);
            }
            else if (grabberRotater.getPosition() + GrabberRotatorIncrement <= GrabberRotatorUpperLimit) {
                grabberRotater.setPosition(grabberRotater.getPosition() - GrabberRotatorIncrement);
            }
        }
   }

    public void rotateGrabberCounterclockwise(){
        if(!loading.get()) {
            if(Math.abs(grabberRotater.getPosition()-GrabberRotatorLoad)<.001){
                grabberRotater.setPosition(grabberRotater.getPosition() + GrabberRotatorIncrement/2);
            }
            else if (grabberRotater.getPosition() - GrabberRotatorIncrement >= GrabberRotatorLowerLimit) {
                grabberRotater.setPosition(grabberRotater.getPosition() + GrabberRotatorIncrement);
            }
        }
    }

    public void elevManualV2(double joystickValue){
        if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to move elevator while output system is moving--ignored");
            return;
        } else {
            if(Math.abs(joystickValue) < 0.85){
                if(joystickValue<0){
                    teamUtil.log("Elev Manual: " + (-ElevCrawlIncrement));

                    elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() - ElevCrawlIncrement, elevatorMinScoreLevel, elevatorMax)));
                    elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() - ElevCrawlIncrement, elevatorMinScoreLevel, elevatorMax)));
                }else{
                    teamUtil.log("Elev Manual: " + ElevCrawlIncrement);
                    elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() + ElevCrawlIncrement, elevatorMinScoreLevel, elevatorMax)));
                    elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() + ElevCrawlIncrement, elevatorMinScoreLevel, elevatorMax)));
                }
            }
            else{
                if(joystickValue<0){
                    teamUtil.log("Elev Manual: " + (-ElevFastIncrement));
                    elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() - ElevFastIncrement, elevatorMinScoreLevel, elevatorMax)));
                    elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() - ElevFastIncrement, elevatorMinScoreLevel, elevatorMax)));
                }else{
                    teamUtil.log("Elev Manual: " + ElevFastIncrement);
                    elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() + ElevFastIncrement, elevatorMinScoreLevel, elevatorMax)));
                    elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() + ElevFastIncrement, elevatorMinScoreLevel, elevatorMax)));

                }
            }

        }
    }

   public void elevManual(double increment){
       if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
           teamUtil.log("WARNING: Attempt to move elevator while output system is moving--ignored");
           return;
       } else {
           teamUtil.log("Elev Manual: " + increment);
           elevLeft.setTargetPosition((int) (clamp(elevLeft.getCurrentPosition() + increment, elevatorMinScoreLevel, elevatorMax)));
           elevRight.setTargetPosition((int) (clamp(elevRight.getCurrentPosition() + increment, elevatorMinScoreLevel, elevatorMax)));
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
   public void oldStraferManual(boolean left){

       if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
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

   public void moveStraferLeft() {
       if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
           teamUtil.log("WARNING: Attempt to strafe grabber in bad position");
           return;
       } else {
           grabberStrafer.setPosition(StraferLeft);
           lastStraferPosition=1;
       }

   }
   public void moveStraferRight() {
       if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
           teamUtil.log("WARNING: Attempt to strafe grabber in bad position");
           return;
       } else {
           grabberStrafer.setPosition(StraferRight);
           lastStraferPosition=2;

       }
   }
    public void moveStraferMiddle() {
        if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to strafe grabber in bad position");
            return;
        } else {
            grabberStrafer.setPosition(StraferLoad);
            lastStraferPosition=3;

        }
    }
   public void straferManual(boolean left){
       if (moving.get() || loading.get()) { // Output system is already moving in a long running operation
           teamUtil.log("WARNING: Attempt to strafe grabber while output system is moving--ignored");
           return;
       } else {
           if(left){
               if(lastStraferPosition==3){
                   grabberStrafer.setPosition(StraferLoad);
                   lastStraferPosition=2;

               }else{
                   grabberStrafer.setPosition(StraferLeft);
                   lastStraferPosition=1;


               }
           }
           else{//going right
               if(lastStraferPosition==1){
                   grabberStrafer.setPosition(StraferLoad);
                   lastStraferPosition=2;


               }
               else{
                   grabberStrafer.setPosition(StraferRight);
                   lastStraferPosition=3;


               }

           }
       }
   }

    public void straferManualV2 (boolean left, boolean right) {
        if(moving.get()||loading.get()){
            return;
        }
        if (left) {
            teamUtil.log("skewing left");
            if (System.currentTimeMillis() > lastStraferServoSkew+ servoSkewTimeMs && (grabberStrafer.getPosition() + StraferIncrement)<StraferLeft) {
                // skew left
                grabberStrafer.setPosition(grabberStrafer.getPosition()+StraferIncrement);
                teamUtil.log("Position: "+(grabberStrafer.getPosition()+StraferIncrement));

                lastStraferServoSkew = System.currentTimeMillis();
            }
        } else if (right) {
            teamUtil.log("skewing right");
            if (System.currentTimeMillis() > lastStraferServoSkew + servoSkewTimeMs && (grabberStrafer.getPosition() - StraferIncrement)>StraferRight) {
                // skew right
                grabberStrafer.setPosition(grabberStrafer.getPosition()-StraferIncrement);
                lastStraferServoSkew = System.currentTimeMillis();
            }
        }
    }

    public void setServosToLoad() {
        grabberStrafer.setPosition(StraferLoad);
        grabberRotater.setPosition(GrabberRotatorLoad);
        rotaterPosition = rotaterPosition.VERTICAL;
        flipper.setPosition(flipperLoad);
        grabber.setPosition(GrabberOpen);
        teamUtil.pause(500); // Wait long enough for all servos to be in a safe position to go to load
        // TODO LATER: The timing on this can be optimized depending on the starting position of the lift
    }

    public void goToLoadNoElevator() {
        moving.set(true);
        teamUtil.log("Go To Load No Elevator");
         // Move servos to correct position and wait for them to complete
        setServosToLoad();
        teamUtil.log("Go To Load No Elevator-Finished");
        moving.set(false);
        loading.set(true);
    }

    // SAFELY Return the output mechanisms to their position for loading pixels
    public void goToLoad() {
        teamUtil.log("Elev Current position in goToLoad: " + elevLeft.getCurrentPosition());
        lastLevel = Math.round(((float)elevLeft.getCurrentPosition()-(float)elevatorMinScoreLevel)/(float)elevatorScoreInc+1);
        teamUtil.log("lastLevel: " +lastLevel);
        moving.set(true);
        teamUtil.log("Go To Load");

        intake.openLid(); // opens intake lid

        if(Math.abs(grabberStrafer.getPosition()-StraferLoad)>0.12f){
            teamUtil.log("Strafing to Middle");

            grabberStrafer.setPosition(StraferLoad);
            teamUtil.pause(1000); //TODO change recalibrate time
        }
        long timeOutTime1 = System.currentTimeMillis() + 1500;
        if (elevLeft.getCurrentPosition() < elevatorSafeStrafeLevel || elevRight.getCurrentPosition() < elevatorSafeStrafeLevel) {
             // we don't know where the servos are so we need to go up to a safe level to move them
            teamUtil.log("Go To Load: Raising to safe level");
            elevRight.setTargetPosition(elevatorSafeStrafeLevel+50);
            elevLeft.setTargetPosition(elevatorSafeStrafeLevel+50);
            while (teamUtil.keepGoing(timeOutTime1)&&elevLeft.getCurrentPosition() < elevatorSafeStrafeLevel || teamUtil.keepGoing(timeOutTime1)&&elevRight.getCurrentPosition() < elevatorSafeStrafeLevel) {
            }
        }
        // Move servos to correct position and wait for them to complete
        teamUtil.log("Go To Load: Positioning Servos");
        setServosToLoad();

        // Take elevator to the bottom
        teamUtil.log("Go To Load: Running to Bottom");
        elevRight.setTargetPosition(elevatorMin);
        elevLeft.setTargetPosition(elevatorMin); // TODO LATER: Might want to turn motors off once we are at bottom to conserve power for driving, etc.
        long timeOutTime2 = System.currentTimeMillis() + 4000;
        while (teamUtil.keepGoing(timeOutTime2)&&elevLeft.getCurrentPosition() > elevatorMin+10 && elevRight.getCurrentPosition() > elevatorMin+10) {
        }
        elevLeft.setVelocity(0);
        elevRight.setVelocity(0);
        intake.closeLid();
        intake.ready();
        teamUtil.log("Go To Load-Finished");
        moving.set(false);
        loading.set(true);


    }

    public void goToLoadNoWait() {
        if (moving.get()||loading.get()) { // Output system is already moving in a long running operation
            teamUtil.log("states in go to load");
            teamUtil.log("moving" + moving.get());
            teamUtil.log("loading" + loading.get());
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
    public void goToScore(float level, double rotatorPosition,double straferPosition) {
        //Reset encoders so they are in unison
        if(teamUtil.alliance == teamUtil.alliance.RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.GOTOSCORE_RED);
        }
        else{
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.GOTOSCORE_BLUE);
        }

        elevLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moving.set(true);
        loading.set(false);

        int elevDestination = (int)(elevatorMinScoreLevel + (level-1f)*elevatorScoreInc);
        teamUtil.log("level" + level);
        teamUtil.log("elev Destination " + elevDestination);

        elevLeft.setVelocity(elevatorMaxVelocity);
        elevRight.setVelocity(elevatorMaxVelocity);
        teamUtil.log("Go To Score");

        intake.openLid(); //opens the hatch at the top of intake

        if (intake.twoPixelsPresent() == true){
            if (grabber.getPosition() < GrabberOpen + .1) { // grabber is currently open
                grabber.setPosition(GrabberClosed);
                teamUtil.pause(350);
                if(details){
                    teamUtil.log("Grabbing 2 pixels");
                }
            }
        }
        else{
            if(grabber.getPosition()< GrabberOpen + .1) {
                grabber.setPosition(GrabberOnePixel);
                teamUtil.pause(350);
                if(details){
                    teamUtil.log("Grabbing 1 pixel");
                }
            }
            //In theory this would grab either one or two pixels depending on whether or not twoPixelsPresent() is true
        }

        intake.stopIntake();



        elevLeft.setTargetPosition(Math.max(elevDestination,elevatorScoreLevel3));
        elevRight.setTargetPosition(Math.max(elevDestination,elevatorScoreLevel3));


        while (elevLeft.getCurrentPosition() < elevatorSafeFlipRotateLevel || elevRight.getCurrentPosition() < elevatorSafeFlipRotateLevel) {
            teamUtil.pause(50);   // yield some time here so drive control gets enough CPU (especially auto!!)
        }
        grabberStrafer.setPosition(straferPosition);
        flipper.setPosition(flipperScore);
        grabberRotater.setPosition(rotatorPosition);

        elevLeft.setTargetPosition(elevDestination);
        elevRight.setTargetPosition(elevDestination);

        while(Math.abs(elevLeft.getCurrentPosition()-elevDestination)>100||Math.abs(elevRight.getCurrentPosition()-elevDestination)>100){
            teamUtil.pause(50);   // yield some time here so drive control gets enough CPU (especially auto!!)
        }

        teamUtil.log("Go To Score-Finished");
        moving.set(false);
    }


    public void goToScoreNoWait(float level, double rotatorPosition,double straferPosition) {
        if (moving.get()||!loading.get()) { // Output system is already moving in a long running operation
            teamUtil.log("states in go to score");

            teamUtil.log("moving" + moving.get());
            teamUtil.log("loading" + loading.get());

            teamUtil.log("WARNING: Attempt to goToScore while output system is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to Go To Score");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToScore(level,rotatorPosition,straferPosition);
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

