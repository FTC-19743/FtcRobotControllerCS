package org.firstinspires.ftc.teamcode.hibble;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "Hibble Op Mode", group = "LinearOpMode")
public class HibbleOpMode extends LinearOpMode {
    TeamGamepad gamepad;
    HibbleMotors hibbleMotors;
    TouchSensor touch;
    @Override

    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        hibbleMotors = new HibbleMotors();
        hibbleMotors.init();
        teamUtil.init(this);
        waitForStart();
        while (opModeIsActive()) {

            //Test for if limitswitch is triggered
            if(hibbleMotors.isUpperLimitSwitchTriggered()){
                telemetry.addLine("UpperTriggered");
                teamUtil.log("Upper Limit Switch Triggered");
            }
            if(hibbleMotors.isLowerLimitSwitchTriggered()){
                telemetry.addLine("LowerTriggered");
                teamUtil.log("Lower Limit Switch Triggered");
            }

            //moves motors up till magnetic switch is triggered
            if(gamepad1.dpad_up){
                hibbleMotors.moveUpTillLimit();
            }

            if(gamepad1.dpad_down){
                hibbleMotors.moveDownTillLimit();
            }

            //this is manual control over hibble
            gamepad.loop();
            if(Math.abs(gamepad1.left_stick_y) < .1){
                hibbleMotors.moveMotors(0);
            }
            else{
                hibbleMotors.moveMotors(gamepad1.left_stick_y);
            }

            //button press test for hibble

            hibbleMotors.wasDownButtonPressed();
            hibbleMotors.wasUpButtonPressed();
            if(hibbleMotors.downButtonWasPressed == true){
                teamUtil.log("Down Button Was Pressed");
                hibbleMotors.moveDownTillLimit();
            }
            if(hibbleMotors.upButtonWasPressed == true){
                teamUtil.log("Up Button Was Pressed");
                hibbleMotors.moveUpTillLimit();
            }
        }
    }
}
