package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

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
    public double sweeperDirection = 1;

    public double lidOpen = 0.8;
    public double lidClosed = 0.25;

    public double newKnockersReady = .168; // was .16

    public double newKnockersHold = .55;

    public double newKnockersCollectOne = .61; // was .57

    public double newKnockersCollectFull = 0.77; // was .7

    public double flickerUp = 1.0;

    public double flickerDown = 0;

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

    public boolean collecterMoving = false;


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

    public void grabOneTeleop(){
        collecterMoving = true;
        if(!intakeRunning){
            startIntake();
            teamUtil.pause(500);
        }
        collectFull();
        teamUtil.pause(500);
        collecterMoving = false;
        ready();
    }

    public void grabOneTeleopNoWait(){
        if (!collecterMoving){
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    grabOneTeleop();
                }
            });
            thread.start();
        }

    }

    public void grabOnePixel(){
        grabbingOnePixel.set(true);
        collectTopPixel();
        teamUtil.pause(500); // TENATIVE VALUE
        grabbingOnePixel.set(false);
    }

    public void grabOnePixelNoWait(){
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                grabOnePixel();
            }
        });
        thread.start();


    }



    public void grabOnePixelToReady(){
        grabbingOnePixel.set(true);
        collectTopPixel();
        teamUtil.pause(300); // TENATIVE VALUE
        ready();
        teamUtil.pause(250);
        grabbingOnePixel.set(false);
    }

    public void grabOnePixelToReadyNoWait(){
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                grabOnePixelToReady();
                if(!teamUtil.theOpMode.gamepad2.dpad_left){
                   ready();
                }
            }
        });
        thread.start();


    }

    public void grabOnePixelLoop(boolean buttonPressed){
        if(buttonPressed){
            if(grabbingOnePixel.get()){
                teamUtil.log("Grab One Pixel Loop Triggered While already grabbing");
            }else{
                grabOnePixelToReadyNoWait();

            }
        }
    }




    public void automaticGrabTwo(){
        boolean details = true;
        collectTopPixel();
        long startTime = System.currentTimeMillis();

        if(details){
            teamUtil.log("Start Time" + startTime);

        }

        while(bottomPixelPresent() == false && teamUtil.keepGoing(startTime + 2500)){
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
            //todo figure out a failsafe
            sweeper.setPower(0);
            kicker.setPower(.3);  // was .1 then .2
        }
        else{
            //todo figure out failsafe
        }

    }

    public boolean autoGrabOne(){
        boolean details = true;
        collectTopPixel();
        long startTime = System.currentTimeMillis();

        if(details){
            teamUtil.log("Start Time" + startTime);

        }
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
            kicker.setPower(.2);
            return true;
        }
        else{
            //todo figure out failsafe
            teamUtil.log("TIMEOUT");
            return false;
        }


    }

    public void automaticGrabTwoNoWait(){
        teamUtil.log("Auto Grab Two No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                automaticGrabTwo();
            }
        });
        thread.start();
    }

    public void autoGrabOneNoWait(){
        teamUtil.log("Auto Grab One No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoGrabOne();
            }
        });
        thread.start();
    }


    public void autoOff(){
        if(intakeRunning) {
            if(onlyOnePixelPresent()){
                teamUtil.log("only one pixel present");
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.GOLD);
            }
            else if(twoPixelsPresent()){
                teamUtil.log("both pixels present");
                if(lastTimePixelSeen == 0) {
                    lastTimePixelSeen = System.currentTimeMillis();
                }
                else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){
                    intakeRunning = false;
                    sweeper.setPower(0);
                    kicker.setPower(.2);
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
                    openLid();
                }
            }
            else{
                lastTimePixelSeen = 0;
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
        }


        else{
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
        //telemetry.addData("PixelDistance ", "Distance:%.3f", pixelDistance.getDistance(DistanceUnit.MM));
    }


}
