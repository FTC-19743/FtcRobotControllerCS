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
    public CRServo kicker;
    public CRServo sweeper;



    private ColorSensor pixelSensor;
    //public DistanceSensor pixelDistance;

    private long lastTimePixelSeen = 0;

    public static double PIXELSENSORTIME = 300;

    public long grabOneOperationTime=800;

    public AtomicBoolean grabbingOnePixel = new AtomicBoolean(false);
    public double kickerDirection = 1;
    public double sweeperDirection = -1;

    public double leftKnockerStore = 0.72;
    public double leftKnockerSweep = 0.44;
    public double leftKnockerReady = 0.44;
    public double leftKnockerDrop = 0.32;

    public double leftKnockerCollect = 0.19;


    public double rightKnockerStore = 0.25;
    public double rightKnockerSweep = 0.54;
    public double rightKnockerReady = 0.55;
    public double rightKnockerDrop = .68;

    public double rightKnockerCollect = 0.81;


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
        store();
        pixelSensor = hardwareMap.get(RevColorSensorV3.class, "pixelSensor");
        //pixelDistance = hardwareMap.get(DistanceSensor.class, "pixelSensor");

        teamUtil.log("Intake Initialized ");
    }

    public void store() {
        lKnocker.setPosition(leftKnockerStore);
        rKnocker.setPosition(rightKnockerStore);
    }

    public void ready() {
        lKnocker.setPosition(leftKnockerReady);
        rKnocker.setPosition(rightKnockerReady);
    }
    public void collect() {
        lKnocker.setPosition(leftKnockerCollect);
        rKnocker.setPosition(rightKnockerCollect);

    }
    public void startIntake(){
        sweeper.setPower(1*sweeperDirection);
        kicker.setPower(1*kickerDirection);
    }

    public void reverseIntake(){
        sweeper.setPower(-1*sweeperDirection);
        kicker.setPower(.1*kickerDirection);
    }

    public void stopIntake(){
        intakeRunning = false;
        sweeper.setPower(0);
        kicker.setPower(0);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);


    }

    public void toggleIntake(){
        if(!intakeRunning){
            intakeRunning=true;
            sweeper.setPower(1*sweeperDirection);
            kicker.setPower(1*kickerDirection);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED);
        }
        else{
            intakeRunning=false;
            sweeper.setPower(0);
            kicker.setPower(.1);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        }
    }



    public void grabOnePixel(){
        grabbingOnePixel.set(true);
        collect();
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
        collect();
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

        collect();
        teamUtil.pause(500); // TENATIVE VALUE

        ready();
        teamUtil.pause(250); // TENATIVE VALUE

        collect();
        teamUtil.pause(250); // TENATIVE VALUE

        store();

    }

    public void autoOff(){
        if(intakeRunning) {
            if(twoPixelsPresent() == true){
                if(lastTimePixelSeen == 0) {
                    lastTimePixelSeen = System.currentTimeMillis();
                }
                else if(System.currentTimeMillis()-lastTimePixelSeen>PIXELSENSORTIME){
                    intakeRunning = false;
                    sweeper.setPower(0);
                    kicker.setPower(.1);
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
                }
            }
            else{
                lastTimePixelSeen = 0;
            }
        }
    }

    public boolean twoPixelsPresent(){

        //if (pixelSensor.alpha()>2500 || pixelSensor.red()>1500 || pixelSensor.blue()>3000||pixelSensor.green()>2400){
        if (pixelSensor.green()>600){
            return true;
        }


        else{
            return false;
        }


    }
    public void outputTelemetry() {
        telemetry.addData("PixelSensor  ", "RGBA: %d %d %d %d ",
                pixelSensor.red(), pixelSensor.green(), pixelSensor.blue(), pixelSensor.alpha());
        telemetry.addData("TwoPixelsPresent  ", "TF: %b ", twoPixelsPresent());
        //telemetry.addData("PixelDistance ", "Distance:%.3f", pixelDistance.getDistance(DistanceUnit.MM));
    }
}
