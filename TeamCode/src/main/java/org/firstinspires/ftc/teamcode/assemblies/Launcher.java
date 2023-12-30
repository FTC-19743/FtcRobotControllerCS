package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Launcher {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public Servo launcher;

    public static double hold  = 0.12;
    public static double release = 0.28;

    public boolean held = false;

    public Launcher(){
        teamUtil.log("Constructing Launcher");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize(){
        teamUtil.log("Initializing Launcher");
        launcher = hardwareMap.get(Servo.class,"launcher");
        launcher.setPosition(hold);
        held = true;
        teamUtil.log("Launcher Initialized ");
    }
    public void toggleRelease(){
        if(held){
            teamUtil.log("Launcher: Toggle Release");
            launcher.setPosition(release);
            held = false;
        }
        else{
            teamUtil.log("Launcher: Toggle Hold");
            launcher.setPosition(hold);
            held = true;
        }
    }

    public void outputTelemetry() {
        telemetry.addData("Launcher  ", "trigger: %f",  launcher.getPosition());
    }
}

