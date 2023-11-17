package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

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

    public double kickerDirection = 1;
    public double sweeperDirection = -1;

    public double leftKnockerStore = 0.76;

    public double leftKnockerCollect = 0.17;

    public double leftKnockerVertical = 0.42;

    public double rightKnockerStore = 0.06;

    public double rightKnockerCollect = 0.64;

    public double rightKnockerVertical = 0.38;

    public boolean intakeRunning = false;


    public Intake(){
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
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
        lKnocker.setPosition(leftKnockerVertical);
        rKnocker.setPosition(rightKnockerVertical);
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


    }

    public void toggleIntake(){
        if(!intakeRunning){
            intakeRunning=true;
            sweeper.setPower(1*sweeperDirection);
            kicker.setPower(1*kickerDirection);
        }
        else{
            intakeRunning=false;
            sweeper.setPower(0);
            kicker.setPower(.1);
        }
    }

    public void grabOnePixel(){
        collect();
        teamUtil.pause(500); // TENATIVE VALUE
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
                }
            }
            else{
                lastTimePixelSeen = 0;
            }
        }
    }

    public boolean twoPixelsPresent(){
        if (pixelSensor.alpha()>2500 || pixelSensor.red()>1500 || pixelSensor.blue()>3000||pixelSensor.green()>2400){
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
