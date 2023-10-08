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
    public CRServo drawbridgeSpinner;
    public Servo drawbridge;
    public CRServo kicker;
    public CRServo sweeper;
    public static double drawbridgeUp = 1.0; //tentative value
    public static double drawbridgeDown = 0.0; //tentative value
    public static double[] pixelStackLevels = {.1,.2,.3,.4};
    public double kickerDirection = -1; //tentative value
    public double sweeperDirection = -1; //tentative value
    public double drawbridgeSpinnerDirection = -1; //tentative value

    public Intake(){
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        Telemetry telemetry = teamUtil.theOpMode.telemetry;
    }

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void initalize(){
        teamUtil.log("Initializing Arm");
        drawbridgeSpinner = hardwareMap.get(CRServo.class,"drawbridge_spinner");
        sweeper = hardwareMap.get(CRServo.class,"sweeper");
        kicker = hardwareMap.get(CRServo.class,"kicker");
        drawbridge = hardwareMap.get(Servo.class,"drawbridge");
    }

    public void lowerDrawbridge(){
        drawbridge.setPosition(drawbridgeDown);
    }

    public void raiseDrawbridge(){
        drawbridge.setPosition(drawbridgeUp);
    }

    public void drawbridgePixelStack(int level){
        if(level < 2 || level > 5){
            lowerDrawbridge();
        }else{
            drawbridge.setPosition(pixelStackLevels[level+1]);
        }
    }

    public void startIntake(){
        drawbridgeSpinner.setPower(1*drawbridgeSpinnerDirection);
        sweeper.setPower(1*sweeperDirection);
        kicker.setPower(1*kickerDirection);
    }

    public void stopIntake(){
        drawbridgeSpinner.setPower(0);
        sweeper.setPower(0);
        kicker.setPower(0);
    }
}
