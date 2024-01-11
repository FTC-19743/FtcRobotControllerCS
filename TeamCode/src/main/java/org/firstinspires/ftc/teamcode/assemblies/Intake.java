package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    public CRServo sweeper;



    private ColorSensor pixelSensorTop;
    private ColorSensor pixelSensorBottom;
    //public DistanceSensor pixelDistance;

    private long lastTimePixelSeen = 0;

    public static double PIXELSENSORTIME = 300;

    public long grabOneOperationTime=800;

    public AtomicBoolean grabbingOnePixel = new AtomicBoolean(false);
    public double kickerDirection = 1;
    public double sweeperDirection = -1;

    public double lidOpen = 0.8;
    public double lidClosed = 0.25;
    public double leftKnockerFullCollect = 0.05;


    public double leftKnockerStore = 0.68;
    public double leftKnockerSweep = 0.44;
    public double leftKnockerReady = 0.44;
    public double leftKnockerDrop = 0.32;

    public double leftKnockerCollect = 0.16;
    public double leftKnockerCollectAuto = 0.19;

    public double rightKnockerFullCollect = 0.93;
    public double rightKnockerStore = 0.29;
    public double rightKnockerSweep = 0.54;
    public double rightKnockerReady = 0.55;
    public double rightKnockerDrop = .68;

    public double rightKnockerCollect = 0.84;
    public double rightKnockerCollectAuto = 0.81;


    public boolean intakeRunning = false;


    public Intake(){
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize(){
        teamUtil.log("Initializing Intake");
        sweeper = hardwareMap.get(CRServo.class,"sweeper");
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
    public void collectTeleop() {
        lKnocker.setPosition(leftKnockerCollect);
        rKnocker.setPosition(rightKnockerCollect);

    }
    public void collectAuto() {
        lKnocker.setPosition(leftKnockerCollectAuto);
        rKnocker.setPosition(rightKnockerCollectAuto);

    }

    public void collectFull(){
        lKnocker.setPosition(leftKnockerFullCollect);
        rKnocker.setPosition(rightKnockerFullCollect);
    }

    public void startIntake(){
        sweeper.setPower(1*sweeperDirection);
        kicker.setPower(1*kickerDirection);
    }

    public void reverseIntake(){
        sweeper.setPower(-1*sweeperDirection);
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
            sweeper.setPower(1*sweeperDirection);
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
        collectTeleop();
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
        collectTeleop();
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
