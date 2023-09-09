package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TeamGamepad {
    public Gamepad gamepad;
    Telemetry telemetry;

    boolean aWasPressed = false;
    boolean aWasPressedLastTime = false;
    boolean aBumpToDo = false;
    boolean bWasPressed = false;
    boolean bWasPressedLastTime = false;
    boolean bBumpToDo = false;
    boolean xWasPressed = false;
    boolean xWasPressedLastTime = false;
    boolean xBumpToDo = false;
    boolean yWasPressed = false;
    boolean yWasPressedLastTime = false;
    boolean yBumpToDo = false;
    boolean upWasPressed = false;
    boolean upWasPressedLastTime = false;
    boolean upBumpToDo = false;
    boolean downWasPressed = false;
    boolean downWasPressedLastTime = false;
    boolean downBumpToDo = false;
    boolean rightWasPressed = false;
    boolean rightWasPressedLastTime = false;
    boolean rightBumpToDo = false;
    boolean leftWasPressed = false;
    boolean leftWasPressedLastTime = false;
    boolean leftBumpToDo = false;
    boolean rightBumperWasPressed = false;
    boolean rightBumperWasPressedLastTime = false;
    boolean rightBumperBumpToDo = false;
    boolean leftBumperWasPressed = false;
    boolean leftBumperWasPressedLastTime = false;
    boolean leftBumperBumpToDo = false;
    double leftTriggerWasPressedLastTime = 0;
    double leftTriggerWasPressed = 0;
    boolean leftTriggerBumpToDo = false;
    boolean optionsWasPressedLastTime = false;
    boolean optionsWasPressed = false;
    boolean optionsBumpToDo = false;

    public TeamGamepad(){

    }
    public void initilize(boolean gamepad1){
        if(gamepad1) {
            gamepad = teamUtil.theOpMode.gamepad1;
        }else{
            gamepad = teamUtil.theOpMode.gamepad2;
        }
        telemetry = teamUtil.telemetry;
    }
    public void loop(){
        aWasPressedLastTime = aWasPressed;
        aWasPressed = gamepad.a;
        bWasPressedLastTime = bWasPressed;
        bWasPressed = gamepad.b;
        xWasPressedLastTime = xWasPressed;

        xWasPressed = gamepad.x;
        yWasPressedLastTime = yWasPressed;
        yWasPressed = gamepad.y;
        downWasPressedLastTime = downWasPressed;
        downWasPressed = gamepad.dpad_down;
        upWasPressedLastTime = upWasPressed;
        upWasPressed = gamepad.dpad_up;
        leftWasPressedLastTime = leftWasPressed;
        leftWasPressed = gamepad.dpad_left;
        rightWasPressedLastTime = rightWasPressed;
        rightWasPressed = gamepad.dpad_right;
        rightBumperWasPressedLastTime = rightBumperWasPressed;
        rightBumperWasPressed = gamepad.right_bumper;
        leftBumperWasPressedLastTime = leftBumperWasPressed;
        leftBumperWasPressed = gamepad.left_bumper;
        leftTriggerWasPressedLastTime = leftTriggerWasPressed;
        leftTriggerWasPressed = gamepad.left_trigger;
        optionsWasPressedLastTime = optionsWasPressed;
        optionsWasPressed = gamepad.options;

        if (aWasPressed == false && aWasPressedLastTime == true) {
            aBumpToDo = true;
        }
        if (bWasPressed == false && bWasPressedLastTime == true) {
            bBumpToDo = true;
        }
        if (xWasPressed == false && xWasPressedLastTime == true) {
            xBumpToDo = true;
        }
        if (yWasPressed == false && yWasPressedLastTime == true) {
            yBumpToDo = true;
        }
        if (upWasPressed == false && upWasPressedLastTime == true) {
            upBumpToDo = true;
        }
        if (downWasPressed == false && downWasPressedLastTime == true) {
            downBumpToDo = true;
        }
        if (leftWasPressed == false && leftWasPressedLastTime == true) {
            leftBumpToDo = true;
        }
        if (rightWasPressed == false && rightWasPressedLastTime == true) {
            rightBumpToDo = true;
        }
        if (rightBumperWasPressed == false && rightBumperWasPressedLastTime == true) {
            rightBumperBumpToDo = true;
        }
        if (leftBumperWasPressed == false && leftBumperWasPressedLastTime == true) {
            leftBumperBumpToDo = true;
        }
        if (leftTriggerWasPressed < 0.8 && leftTriggerWasPressedLastTime >= 0.8){
            leftTriggerBumpToDo = true;
        }
        if (optionsWasPressed == false && optionsWasPressedLastTime == true) {
            optionsBumpToDo = true;
        }
    }
    public boolean wasAPressed(){
        if(aBumpToDo) {
            aBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasBPressed(){
        if(bBumpToDo) {
            bBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasXPressed(){
        if(xBumpToDo) {
            xBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasYPressed(){
        if(yBumpToDo) {
            yBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasUpPressed(){
        if(upBumpToDo) {
            upBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasDownPressed(){
        if(downBumpToDo) {
            downBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightPressed(){
        if(rightBumpToDo) {
            rightBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftPressed(){
        if(leftBumpToDo) {
            leftBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasRightBumperPressed(){
        if(rightBumperBumpToDo) {
            rightBumperBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftBumperPressed(){
        if(leftBumperBumpToDo) {
            leftBumperBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasLeftTriggerPressed(){
        if(leftTriggerBumpToDo){
            leftTriggerBumpToDo = false;
            return true;
        }
        return false;
    }
    public boolean wasOptionsPressed(){
        if(optionsBumpToDo){
            optionsBumpToDo = false;
            return true;
        }
        return false;
    }
}

