package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo lKnocker;
    public Servo rKnocker;
    public CRServo kicker;
    public CRServo sweeper;
    public static double[] pixelStackLevels = {.1,.2,.3,.4};
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
        kicker.setPower(-1*kickerDirection);
    }

    public void stopIntake(){
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
    public void outputTelemetry() {

    }
}
