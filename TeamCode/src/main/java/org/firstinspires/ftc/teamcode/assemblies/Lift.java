package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

public class Lift {
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public static double liftArmUp = 0.71;
    public static double liftArmStowed = 0.30;
    public static double liftSpindleVelocity = 3000; // Max Speed
    public Servo liftArm;
    private DcMotorEx liftSpindle;

    public boolean armUp = false;

    public Lift(){
        teamUtil.log("Constructing Lift");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize(){
        teamUtil.log("Initializing Lift");
        liftSpindle = hardwareMap.get(DcMotorEx.class, "liftSpindle");
        liftArm = hardwareMap.get(Servo.class,"liftArm");
        liftArm.setPosition(liftArmStowed);
        teamUtil.log("Lift Initialized ");
    }
    public void toggleArm(){
        if(!armUp){
            liftArm.setPosition(liftArmUp);
            armUp=true;
        }
        else{
            liftArm.setPosition(liftArmStowed);
            armUp=false;
        }
    }
    public void armUp() {
        liftArm.setPosition(liftArmUp);
    }
    public void stowArm(){
        liftArm.setPosition(liftArmStowed);
    }
    public void raiseLift(){
        liftSpindle.setVelocity(liftSpindleVelocity);
    }
    public void lowerLift(){
        liftSpindle.setVelocity(-liftSpindleVelocity);
    }
    public void stopLift()
    { // TODO: Might want to switch to Run TO Position and set target to current position.
        liftSpindle.setVelocity(0);
    }

    public void outputTelemetry() {
        telemetry.addData("Lift  ", "Motor: %d, Arm: %f", liftSpindle.getCurrentPosition(), liftArm.getPosition());
    }
}
