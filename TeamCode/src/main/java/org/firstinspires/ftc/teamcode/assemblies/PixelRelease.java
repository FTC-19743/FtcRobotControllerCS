package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class PixelRelease {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public static double release = 0.9;
    public static double hold = 0.30;
    public Servo releaseServo;
    boolean holding = false;

    public PixelRelease(){
        teamUtil.log("Constructing PixelRelease");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize(){
        teamUtil.log("Initializing PixelRelease");
        releaseServo = hardwareMap.get(Servo.class,"pixelrelease");
        //releaseServo.setPosition(release);
        holding = false;
        teamUtil.log("PixelRelease Initialized ");
    }
    public void toggle(){
        if(!holding){
            teamUtil.log("Hold Pixel");
            releaseServo.setPosition(hold);
            holding=true;
        }
        else{
            teamUtil.log("Release Pixel");
            releaseServo.setPosition(release);
            holding=false;
        }
    }
    public void release() {
        teamUtil.log("Release Pixel");
        releaseServo.setPosition(release);
        holding=false;
    }
    public void hold(){
        teamUtil.log("Hold Pixel");
        releaseServo.setPosition(hold);
        holding=true;
    }

}
