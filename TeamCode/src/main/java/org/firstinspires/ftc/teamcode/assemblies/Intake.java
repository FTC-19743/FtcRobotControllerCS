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

    public double leftKnockerReady = 0.833; // was .9 was .74

    public double leftKnockerHold = .56;
    public double leftKnockerGetTop = 0.5; //was .3
    public double leftKnockerGetBottom = 0.45; //was .25
    public double leftKnockerCollect = leftKnockerGetTop; // was 0.16
    public double leftKnockerFullCollect = leftKnockerGetBottom; // was .05
    public double leftKnockerCollectAuto = leftKnockerGetTop; // was 0.19
    public double leftKnockerStore = leftKnockerGetTop; // was .68

    public double rightKnockerReady = 0.178; // was .11  was .25 was .55

    public double rightKnockerHold = .43;
    public double rightKnockerGetTop = 0.50; // was .7
    public double rightKnockerGetBottom = 0.55; // was.73
    public double rightKnockerCollect = rightKnockerGetTop; // was 0.84
    public double rightKnockerFullCollect = rightKnockerGetBottom; // was 0.93
    public double rightKnockerCollectAuto = rightKnockerGetTop; // was 0.81
    public double rightKnockerStore = rightKnockerGetTop; // was 0.29

    public boolean intakeRunning = false;


    public Intake(){
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize(){
        teamUtil.log("Initializing Intake");
        sweeper = hardwareMap.get(DcMotorEx.class,"sweeper");
        sweeper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kicker = hardwareMap.get(CRServo.class,"kicker");

        rKnocker = hardwareMap.get(Servo.class,"rKnocker");
        lKnocker = hardwareMap.get(Servo.class,"lKnocker");
        pixelLid = hardwareMap.get(Servo.class,"pixellid");

        store();
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
    public void store() {
        lKnocker.setPosition(leftKnockerStore);
        rKnocker.setPosition(rightKnockerStore);
    }

    public void ready() {
        lKnocker.setPosition(leftKnockerReady);
        rKnocker.setPosition(rightKnockerReady);
    }
    public void collectTopPixel() {
        lKnocker.setPosition(leftKnockerCollect);
        rKnocker.setPosition(rightKnockerCollect);

    }
    public void collectAuto() {
        lKnocker.setPosition(leftKnockerCollectAuto);
        rKnocker.setPosition(rightKnockerCollectAuto);

    }

    public void collectHold(){
        lKnocker.setPosition(leftKnockerHold);
        rKnocker.setPosition(rightKnockerHold);
    }


    public void holdToCollect(){
        startIntake();
        teamUtil.pause(500); //tentative time
        collectTopPixel();
        teamUtil.pause(1000);
        collectFull();
    }

    public void collectFull(){
        lKnocker.setPosition(leftKnockerFullCollect);
        rKnocker.setPosition(rightKnockerFullCollect);
    }

    public void startIntake(){
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



    public void grabOnePixel(){
        grabbingOnePixel.set(true);
        collectTopPixel();
        teamUtil.pause(500); // TENATIVE VALUE
        store();
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
                    store();
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


    public void grabTwoPixels(){

        collectAuto();
        teamUtil.pause(300); // TENATIVE VALUE

        ready();
        teamUtil.pause(350); // TENATIVE VALUE

        collectFull();
        teamUtil.pause(350); // TENATIVE VALUE

        //store();

    }

    public void autoGrabTwo(){
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
            teamUtil.log("Bottom Pixel Present" + bottomPixelPresent());
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
            kicker.setPower(.1);
        }
        else{
            //todo figure out failsafe
        }

    }

    public void autoGrabTwoNoWait(){
        teamUtil.log("Auto Grab Two No Wait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                autoGrabTwo();
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
                    kicker.setPower(.1);
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
