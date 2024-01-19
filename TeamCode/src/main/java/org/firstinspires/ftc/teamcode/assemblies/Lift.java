package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

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
    public DcMotorEx liftSpindle;

    public boolean startedLifting = false;

    public boolean armUp = false;

    public Lift(){
        teamUtil.log("Constructing Lift");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize(){
        teamUtil.log("Initializing Lift");
        liftSpindle = hardwareMap.get(DcMotorEx.class, "liftAndStrafeEncoder");
        liftArm = hardwareMap.get(Servo.class,"liftArm");
        liftArm.setPosition(liftArmStowed);
        armUp = false;
        teamUtil.log("Lift Initialized ");
    }
    public void toggleArm(){
        if(!armUp){
            teamUtil.log("Lift: Arm Up");
            liftArm.setPosition(liftArmUp);
            armUp=true;
        }
        else{
            teamUtil.log("Lift: Arm Down");
            liftArm.setPosition(liftArmStowed);
            armUp=false;
        }
    }
    public void armUp() {
        teamUtil.log("Lift: Arm Up");
        liftArm.setPosition(liftArmUp);
        armUp=true;
    }
    public void stowArm(){
        teamUtil.log("Lift: Arm Down");
        liftArm.setPosition(liftArmStowed);
        armUp=false;
    }
    public void raiseLift(){
        // TODO: Needs to be reimplemented to use setPower and no encoder
        /*
        liftSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        startedLifting = true;
        liftSpindle.setVelocity(liftSpindleVelocity);
         */
    }
    public void lowerLift(){
        // TODO: Needs to be reimplemented to use setPower and no encoder
        /*
                liftSpindle.setVelocity(-liftSpindleVelocity);
         */
    }
    public void stopLift()
    {
        // TODO: Needs to be reimplemented to use setPower and no encoder
        /*
                liftSpindle.setVelocity(0);
         */
    }
    public void holdLift(){
        // TODO: Needs to be reimplemented to use setPower and no encoder
        /*
                liftSpindle.setTargetPosition(liftSpindle.getCurrentPosition());
        liftSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftSpindle.setVelocity(3000);
         */
    }
    public void outputTelemetry() {
        telemetry.addData("Lift  ", "Motor: %d, Arm: %f", liftSpindle.getCurrentPosition(), liftArm.getPosition());
    }
}
