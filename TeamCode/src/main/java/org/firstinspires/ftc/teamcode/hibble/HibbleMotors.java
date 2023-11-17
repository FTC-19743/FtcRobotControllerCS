package org.firstinspires.ftc.teamcode.hibble;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class HibbleMotors {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    TouchSensor touch;

    public TouchSensor upperLimit;
    public TouchSensor lowerLimit;
    public TouchSensor upButton;
    public TouchSensor downButton;
    public DcMotorEx motorZero;
    public DcMotorEx motorOne;

    boolean upButtonWasPressed = false;
    boolean upButtonWasPressedLastTime = false;
    boolean upButtonBumpToDo = false;

    boolean downButtonWasPressed = false;
    boolean downButtonWasPressedLastTime = false;
    boolean downButtonBumpToDo = false;


    public HibbleMotors() {
        teamUtil.log("Constructing Hibble Motors");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;

    }

    public void init() {
        teamUtil.log("Initializing Arm");
        motorZero = hardwareMap.get(DcMotorEx.class, "motor_zero");
        motorOne = hardwareMap.get(DcMotorEx.class, "motor_one");
        motorZero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initializes Limit Switches
        upperLimit = hardwareMap.get(TouchSensor.class, "upper_limit");
        lowerLimit = hardwareMap.get(TouchSensor.class,"lower_limit");
        upButton = hardwareMap.get(TouchSensor.class, "up_button");
        downButton = hardwareMap.get(TouchSensor.class, "down_button");

    }

    public void moveMotors(double power){
        motorZero.setPower(power);
        motorOne.setPower(power);
    }

    public void moveUpTillLimit(){
        moveMotors(-1);
        while(!isUpperLimitSwitchTriggered()){}
        moveMotors(0);
    }

    public void moveDownTillLimit(){
        moveMotors(1);
        while(!isLowerLimitSwitchTriggered()){}
        moveMotors(0);
    }

    public boolean isUpperLimitSwitchTriggered(){
        if (upperLimit.isPressed()){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean isLowerLimitSwitchTriggered(){
        if(lowerLimit.isPressed()){
            return true;
        }
        else{
            return false;
        }
    }

    public void wasDownButtonPressed(){
        if(downButtonWasPressedLastTime == true && downButton.isPressed() == false){
            downButtonWasPressed = true;
            downButtonWasPressedLastTime = false;
        }
        else if(downButton.isPressed() == true){
            downButtonWasPressed = false;
            downButtonWasPressedLastTime = true;
        }
        else{
            downButtonWasPressed = false;
            downButtonWasPressedLastTime = false;
        }
    }

    public void wasUpButtonPressed(){
        if(upButtonWasPressedLastTime == true && upButton.isPressed() == false){
            upButtonWasPressed = true;
            upButtonWasPressedLastTime = false;
        }
        else if(upButton.isPressed() == true){
            upButtonWasPressed = false;
            upButtonWasPressedLastTime = true;
        }
        else{
            upButtonWasPressed = false;
            upButtonWasPressedLastTime = false;
        }
    }


}
