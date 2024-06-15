package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo lKnocker;
    public Servo rKnocker;

    public Servo flicker;

    public Servo knockers;
    public Servo pixelLid;

    public CRServo kicker;
    public DcMotorEx sweeper;



    private ColorSensor pixelSensorTop;
    private ColorSensor pixelSensorBottom;
    //public DistanceSensor pixelDistance;

    private long lastTimePixelSeen = 0;

    public static double PIXELSENSORTIME = 300;

    public long grabOneOperationTime=800;

    public AtomicBoolean grabbingOnePixel = new AtomicBoolean(false);
    public double kickerDirection = 1;
    public double kickerAutoGrabSpeed = 0.12;
    public double sweeperDirection = 1;

    public double lidOpen = 0.8;
    public double lidClosed = 0.25;

    public double newKnockersReady = .177; // was .168

    public double newKnockersHold = .55;

    public double newKnockersCollectOne = .61; // was .57

    public double newKnockersCollectFull = 0.72; // was .7 then .77

    public double flickerUp = 0.99; //was .99
    public double flickerDown = .83;
    public double flickerPos[] = {0.99,0.79, .59, .39, .18, .02}; //first value was .99
    public int currentFlickerPos = 0;
    public int maxFlickerPos = 5;

    public double leftKnockerReady = 0.833; // was .9 was .74

    public double leftKnockerHold = .56;
    public double leftKnockerGetTop = 0.50; //was .5
    public double leftKnockerGetBottom = 0.45; //was .25
    public double leftKnockerCollect = leftKnockerGetTop; // was 0.16
    public double leftKnockerFullCollect = leftKnockerGetBottom; // was .05
    public double leftKnockerCollectAuto = leftKnockerGetTop; // was 0.19
    public double leftKnockerStore = leftKnockerGetTop; // was .68

    public double rightKnockerReady = 0.178; // was .11  was .25 was .55

    public double rightKnockerHold = .43;
    public double rightKnockerGetTop = 0.49; // was .7 was .5
    public double rightKnockerGetBottom = 0.55; // was.73
    public double rightKnockerCollect = rightKnockerGetTop; // was 0.84
    public double rightKnockerFullCollect = rightKnockerGetBottom; // was 0.93
    public double rightKnockerCollectAuto = rightKnockerGetTop; // was 0.81
    public double rightKnockerStore = rightKnockerGetTop; // was 0.29

    public boolean intakeRunning = false;

    public boolean flicking = false;

    public boolean collecterMoving = false;
    public int pixelsLoaded = 0;


    public Intake(){
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize(){
        teamUtil.log("Initializing Intake");
        sweeper = hardwareMap.get(DcMotorEx.class,"sweeperAndForwardEncoder");
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kicker = hardwareMap.get(CRServo.class,"kicker");
        knockers = hardwareMap.get(Servo.class,"knockers");
        flicker = hardwareMap.get(Servo.class,"flicker");
        ServoControllerEx flickerControl = (ServoControllerEx) flicker.getController();
        int flickerPort = flicker.getPortNumber();
        PwmControl.PwmRange flickerRange = new PwmControl.PwmRange(500, 2500);
        flickerControl.setServoPwmRange(flickerPort, flickerRange);
        //rKnocker = hardwareMap.get(Servo.class,"rKnocker");
        //lKnocker = hardwareMap.get(Servo.class,"lKnocker");
        pixelLid = hardwareMap.get(Servo.class,"pixellid");

        ready();
        flicker.setPosition(flickerUp);
        openLid();

        pixelSensorTop = hardwareMap.get(RevColorSensorV3.class, "pixelSensor");
        pixelSensorBottom = hardwareMap.get(RevColorSensorV3.class, "onePixelSensor");

        //pixelDistance = hardwareMap.get(DistanceSensor.class, "pixelSensor");

        teamUtil.log("Intake Initialized ");
    }

    public void openLid () {
        pixelLid.setPosition(lidOpen);
    }
    public void closeLid() {
        pixelLid.setPosition(lidClosed);
    }


    public void ready() {
        if (!collecterMoving){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);
            knockers.setPosition(newKnockersReady);
        }
    }
    public void collectTopPixel() {

        knockers.setPosition(newKnockersCollectOne);
        /*
        lKnocker.setPosition(leftKnockerCollect);
        rKnocker.setPosition(rightKnockerCollect);

         */

    }


    public void collectHold(){
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.JUDGING_BLINKIN);
        knockers.setPosition(newKnockersHold);
        /*
        lKnocker.setPosition(leftKnockerHold);
        rKnocker.setPosition(rightKnockerHold);

         */
    }


    public void holdToCollect(long timeout){
        long startTime = System.currentTimeMillis();
        collecterMoving = true;
        startIntake();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED);
        teamUtil.pause(500); //tentative time
        collectTopPixel();

        while(bottomPixelPresent() == false && teamUtil.keepGoing(startTime + timeout)){
            teamUtil.pause(50);
        }
        // TODO add failsafe
        collectFull();
        collecterMoving = false;

    }

    public void holdToCollectNoWait(long timeout){
        if (collecterMoving){

        }
        else{
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    holdToCollect(timeout);
                }
            });
            thread.start();
        }

    }
    public void collectFull(){
        knockers.setPosition(newKnockersCollectFull);
        /*
        lKnocker.setPosition(leftKnockerFullCollect);
        rKnocker.setPosition(rightKnockerFullCollect);

         */
    }
    public void testWithFlicker(){
        startIntake();
        teamUtil.pause(500);
        putFlickerDown();
        teamUtil.pause(500);
        knockers.setPosition(newKnockersCollectFull);
        while(!onlyOnePixelPresent()){
        }
        putFlickerUp();
        knockers.setPosition(newKnockersReady);


    }

    public void putFlickerUp(){
        flicker.setPosition(flickerUp);
    }

    public void putFlickerDown(){
        flicker.setPosition(flickerDown);
    }

    public boolean canFlickAgain() {
        return currentFlickerPos< maxFlickerPos;
    };
    public void flick() {
        currentFlickerPos++;
        flicker.setPosition(flickerPos[currentFlickerPos]);
    }
    public void resetFlicker() {
        currentFlickerPos = 0;
        flicker.setPosition(flickerPos[currentFlickerPos]);
    }
    public void startIntake(){
        intakeRunning = true;
        sweeper.setPower(0.3*sweeperDirection);
        kicker.setPower(1*kickerDirection);
    }

    public void reverseIntake(){
        sweeper.setPower(-0.1*sweeperDirection);
        kicker.setPower(-1*kickerDirection);
    }

    public void stopIntake(){
        teamUtil.log("Stopping Intake");
        intakeRunning = false;
        sweeper.setPower(0);
        kicker.setPower(0);
    }

    public void toggleIntake(){
        if(!intakeRunning){
            intakeRunning=true;
            sweeper.setPower(0.1*sweeperDirection);
            kicker.setPower(1*kickerDirection);
            closeLid();
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED);
        }
        else{
            intakeRunning=false;
            sweeper.setPower(0);
            kicker.setPower(.1);
            openLid();
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        }
    }

    public void grabOnePixel(){

    }
    public void teleopFlickOne(){
        if(!twoPixelsPresent())
        {
            putFlickerDown();
            teamUtil.pause(300);
            teleopGetOne();
            putFlickerUp();
        }
    }
    public void teleopFlickOneV2(){
        if(!twoPixelsPresent() && canFlickAgain()&&!grabbingOnePixel.get()&&!flicking)
        {
            flicking = true;
            flick();
            teamUtil.pause(300);
            teleopGetOne();
            flicking=false;
        }
    }




//    public void grabOnePixel(){
//        grabbingOnePixel.set(true);
//        collectTopPixel();
//        teamUtil.pause(500); // TENATIVE VALUE
//        grabbingOnePixel.set(false);
//    }
//
    public void teleopFlickOneNoWait(){
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                teleopFlickOneV2();
            } // TODO: V2 is experimental
        });
        thread.start();


    }



    public void teleopGetOne(){
        if(!grabbingOnePixel.get()){
            grabbingOnePixel.set(true);
            long startTime = System.currentTimeMillis();
            if(!twoPixelsPresent()){
                int pixelsPresent = 0;
                if(bottomPixelPresent()){
                    pixelsPresent = 1;
                }
                collectFull();
                if (pixelsPresent == 0){
                    while(bottomPixelPresent() == false && teamUtil.keepGoing(startTime + 1000)){
                        teamUtil.pause(50);
                    }
                }
                else{
                    while(twoPixelsPresent() == false && teamUtil.keepGoing(startTime + 1000)){
                        teamUtil.pause(50);
                    }
                }
                ready();
                if(System.currentTimeMillis()-startTime>1500){
                    teamUtil.log("teleopOnePixel TIMED OUT");
                }
            }
            grabbingOnePixel.set(false);
        }
        else{
            teamUtil.log("Tele Op Get One Failed because Collectors already moving");
        }

    }

    public void teleopGetOneNoWait(){
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                teleopGetOne();
                if(!teamUtil.theOpMode.gamepad2.dpad_left){
                   ready();
                }
            }
        });
        thread.start();


    }

//    public void grabOnePixelLoop(boolean buttonPressed){
//        if(buttonPressed){
//            if(grabbingOnePixel.get()){
//                teamUtil.log("Grab One Pixel Loop Triggered While already grabbing");
//            }else{
//                grabOnePixelToReadyNoWait();
//
//            }
//        }
//    }

    public void teleopGetTwo(){
        boolean details = true;
        long startTime = System.currentTimeMillis();

        if(details){
            teamUtil.log("Start Time" + startTime);
        }

        if(bottomPixelPresent()){ //Checks if there are one or two pixels present if so it does not run
            teamUtil.log("Pixels were present so we FAILSAFED");
            return;
        } else{
            collectTopPixel();
            while(bottomPixelPresent() == false && teamUtil.keepGoing(startTime + 1500)){
                teamUtil.pause(50);
            }
            if(details){
                teamUtil.log("Bottom Pixel Present: " + bottomPixelPresent());
                teamUtil.log("Time after first pixel Collection" + System.currentTimeMillis());
            }
            if(bottomPixelPresent()){
                collectFull();
                teamUtil.pause(500);
                while(twoPixelsPresent() == false && teamUtil.keepGoing(startTime + 1000)) {
                    teamUtil.pause(50);
                }
                if(!twoPixelsPresent()){
                    ready();
                    teamUtil.log("We TIMED OUT with one pixel loaded");
                }
            }
            else{
                ready();
                teamUtil.log("We TIMED OUT");
            }
        }


    }
    public void autoLoadTwoPixelsFromStack(){
        boolean details = true;
        long startTime = System.currentTimeMillis();

        if(details){
            teamUtil.log("Start Time" + startTime);
        }

            collectTopPixel();
            while(bottomPixelPresent() == false && teamUtil.keepGoing(startTime + 1500)){
                teamUtil.pause(50);
            }
            if(details){
                teamUtil.log("Bottom Pixel Present: " + bottomPixelPresent());
            }
            if(bottomPixelPresent()){
                collectFull();
                teamUtil.pause(500);
                while(twoPixelsPresent() == false && teamUtil.keepGoing(startTime + 1000)) {
                    teamUtil.pause(50);
                }
            }
            else{
                ready();
                teamUtil.log("We TIMED OUT");
            }
    }

    public AtomicBoolean doneLoading = new AtomicBoolean(false);
    public void autoLoadTwoPixelsFromStackV2(){
        teamUtil.log("autoLoadTwoPixelsFromStackV2");
        doneLoading.set(false);
        boolean details = true;
        long startTime = System.currentTimeMillis();

        collectTopPixel();
        while(!twoPixelsPresent() && !bottomPixelPresent() && teamUtil.keepGoing(startTime + 1500)){
            teamUtil.pause(50);
        }
        if(!bottomPixelPresent() && !twoPixelsPresent()) {
            teamUtil.log("TIMED OUT while loading first pixel");
            ready();
        } else {
            if(details) teamUtil.log("Loaded one");
            collectFull();
            lastTimePixelSeen = 0;
            while(teamUtil.keepGoing(startTime+3000)){
                if(twoPixelsPresent()){
                    if(lastTimePixelSeen == 0) {
                        lastTimePixelSeen = System.currentTimeMillis();
                    }
                    else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){
                        break;
                    }
                }
                else{
                    lastTimePixelSeen = 0;
                }
                teamUtil.pause(50);
            }
            if(!twoPixelsPresent()) {
                teamUtil.log("TIMED OUT while loading second pixel");
                ready();
            } else {
                if(details) teamUtil.log("Loaded two");
                teamUtil.pause(200); // allow a little extra time to make sure both pixels are fully loaded
                kicker.setPower(kickerAutoGrabSpeed);
                sweeper.setPower(0);
                teamUtil.robot.output.grabPixels();
                openLid();
                teamUtil.pause(350);
                if(details) teamUtil.log("Grabbed two");
                doneLoading.set(true); // signal that we have successfully loaded and grabbed two pixels
            }
        }
    }


    public boolean autoLoadSecondPixelFromStack(){ // assumes we already have 1 pixel loaded
        boolean details = true;
        collectTopPixel();
        long startTime = System.currentTimeMillis();

        if(details){
            teamUtil.log("Start Time" + startTime);

        }
        lastTimePixelSeen = 0; // TODO: This was uninitialized, which means if we ran auto 2+ times, the timing loop below would have not worked.  Not sure why it is needed?
        while(teamUtil.keepGoing(startTime+2500)){
            if(twoPixelsPresent()){
                if(lastTimePixelSeen == 0) {
                    lastTimePixelSeen = System.currentTimeMillis();
                }
                else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){
                    break;
                }

            }
            else{
                lastTimePixelSeen = 0;
            }
            teamUtil.pause(50);

        }


        if(details){
            teamUtil.log("Two pixels Present" + twoPixelsPresent());
            teamUtil.log("Time after pixel Collection" + System.currentTimeMillis());
        }

        if(twoPixelsPresent()){
            sweeper.setPower(0);
            kicker.setPower(kickerAutoGrabSpeed); // was .3
            return true;
        }
        else{
            //todo figure out failsafe
            teamUtil.log("TIMEOUT");
            return false;
        }


    }

    public boolean autoLoadSecondPixelFromStackV2(){ // assumes we already have 1 pixel loaded
        teamUtil.log("autoLoadSecondPixelFromStackV2");
        boolean details = true;
        collectTopPixel();
        long startTime = System.currentTimeMillis();

        while(!twoPixelsPresent() && teamUtil.keepGoing(startTime+2000)){
            teamUtil.pause(50);
        }
        sweeper.setPower(0);
        kicker.setPower(kickerAutoGrabSpeed); // was .3
        if(twoPixelsPresent()) {
            teamUtil.log("Second Pixel Loaded");
            return true;
        } else {
            teamUtil.log("FAILED to load second pixel");
            return false;
        }
    }

    public void teleopGetTwoNoWait(){
        teamUtil.log("Auto Grab Two No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                teleopGetTwo();
            }
        });
        thread.start();
    }
    public void autoLoadTwoPixelsFromStackNoWait(){
        teamUtil.log("Auto Grab Two No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoLoadTwoPixelsFromStackV2();
            }
        });
        thread.start();
    }

    public void autoGrabOneNoWait(){
        teamUtil.log("Auto Grab One No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoLoadSecondPixelFromStack();
            }
        });
        thread.start();
    }


    public void autoOff(){
        if(intakeRunning) {
            if(onlyOnePixelPresent()){
                teamUtil.log("only one pixel present");
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.GOLD);
                pixelsLoaded = 1;
            }
            else if(twoPixelsPresent()){
                teamUtil.log("both pixels present");
                if(lastTimePixelSeen == 0) {
                    lastTimePixelSeen = System.currentTimeMillis();
                }
                else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){
                    intakeRunning = false;
                    sweeper.setPower(0);
                    kicker.setPower(.15 ); //was 0.3
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
                    openLid();
                    pixelsLoaded = 2;
                }
            }
            else{
                lastTimePixelSeen = 0;
                pixelsLoaded = 0;
            }
        }
    }
    public void autoOffV2(){
        if(intakeRunning) {
            if(onlyOnePixelPresent()){
                teamUtil.log("only one pixel present");
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.GOLD);
                pixelsLoaded = 1;
                lastTimePixelSeen = 0;
            }
            else if(twoPixelsPresent()){
                teamUtil.log("both pixels present");
                if(lastTimePixelSeen == 0) {
                    lastTimePixelSeen = System.currentTimeMillis();
                }
                else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){
                    intakeRunning = false;
                    sweeper.setPower(0);
                    kicker.setPower(.15 ); //was 0.3
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
                    openLid();
                    pixelsLoaded = 2;
                }
            }
            else{
                lastTimePixelSeen = 0;
                pixelsLoaded = 0;
            }
        }
    }

    public boolean intakeSlowing= false;
    public long intakeSlowStartTime = 0;
    public long KICKERSLOWTIME = 250;
    public void autoOffV3(){
        if(intakeRunning) {
            if(onlyOnePixelPresent()){ // lower pixel loaded
                teamUtil.log("One pixel present");
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.GOLD);
                pixelsLoaded = 1;
                lastTimePixelSeen = 0;
                intakeSlowing = false;
                intakeSlowStartTime = 0;
            }
            else if(twoPixelsPresent()){ // Top pixel visible (but might be the first pixel bouncing high)
                //teamUtil.log("both pixels present");
                if(lastTimePixelSeen == 0) { // If this is the first time this cycle where we have seen the top pixel, note the current time
                    lastTimePixelSeen = System.currentTimeMillis();
                    teamUtil.log("top pixel present");
                } else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){ // If not the first time, see how long it has been since we first saw the top pixel
                    // Enough time has gone by so slow down the intake
                    if (intakeSlowStartTime==0) { // start slowing down the intake
                        //intakeRunning = false;
                        intakeSlowing = true;
                        intakeSlowStartTime = System.currentTimeMillis(); // note when we started the slowing process
                        sweeper.setPower(0);
                        kicker.setPower(.15);
                        teamUtil.log("Decelerating");
                        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
                        //openLid();
                        //pixelsLoaded = 2;
                    } else if (System.currentTimeMillis()-intakeSlowStartTime>KICKERSLOWTIME){ // if enough time has gone by since we started slowing down
                        intakeRunning = false;
                        intakeSlowing = false;
                        sweeper.setPower(0);
                        kicker.setPower(.1); // slow all the way down
                        teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
                        openLid();
                        pixelsLoaded = 2;
                        teamUtil.log("Running at slowest speed");
                    }
                }
            }
            else{ // No Pixels present
                lastTimePixelSeen = 0;
                pixelsLoaded = 0;
                intakeSlowing = false;
                intakeSlowStartTime = 0;
            }
        }
    }
    public void autoOffLoop(long timeOut){
        long timeOutTime = System.currentTimeMillis() + timeOut;
        while(intakeRunning && teamUtil.keepGoing(timeOutTime)) {
            autoOff();
            teamUtil.pause(50);
        }
    }

    public void autoOffLoopNoWait(long timeOut){
        teamUtil.log("Auto Off Loop No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoOffLoop(timeOut);
            }
        });
        thread.start();
    }

    public boolean onlyOnePixelPresent(){
        if (pixelSensorBottom.green()>1000 && pixelSensorTop.green()<1000){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean bottomPixelPresent(){
        if (pixelSensorBottom.green()>1000){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean twoPixelsPresent(){
        //if (pixelSensor.alpha()>2500 || pixelSensor.red()>1500 || pixelSensor.blue()>3000||pixelSensor.green()>2400){
        if (pixelSensorTop.green()>1000){
            return true;
        } else {
            return false;
        }
    }
    public void outputTelemetry() {
        telemetry.addData("PixelSensorTop  ", "RGBA: %d %d %d %d ",
                pixelSensorTop.red(), pixelSensorTop.green(), pixelSensorTop.blue(), pixelSensorTop.alpha());
        telemetry.addData("PixelSensorTop  ", "RGBA: %d %d %d %d ",
                pixelSensorBottom.red(), pixelSensorBottom.green(), pixelSensorBottom.blue(), pixelSensorBottom.alpha());
        telemetry.addData("OnlyOnePixelPresent: ", "TF: %b", onlyOnePixelPresent());
        telemetry.addData("TwoPixelsPresent  ", "TF: %b ", twoPixelsPresent());
        telemetry.addLine("# Pixels present:  " + pixelsLoaded);
        //telemetry.addData("PixelDistance ", "Distance:%.3f", pixelDistance.getDistance(DistanceUnit.MM));
    }


}
