package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;



@TeleOp(name="Ultrasonic Test", group="Linear Opmode")
@Disabled
public class AxonTest extends LinearOpMode {

    public Servo axon;
    TeamGamepad gamepad;

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    @Override
    public void runOpMode(){


        teamUtil.init(this);
        axon.setPosition(0.5);
        AnalogInput axonPotentiometer;
        axonPotentiometer = hardwareMap.analogInput.get("ult");
        waitForStart();
        while(opModeIsActive()) {
            gamepad = new TeamGamepad();
            gamepad.initilize(true);
            gamepad.loop();

            double voltage = axonPotentiometer.getVoltage();
            double potentiometerDegrees = (260/3*(voltage-.55)+36)*2.54;
            telemetry.addLine("Voltage:  " + voltage);
            telemetry.addLine("Degrees:  "+ potentiometerDegrees);
            telemetry.addLine("Servo Position:  " + axon.getPosition());
            telemetry.update();


            if(gamepad.wasUpPressed()){
                axon.setPosition(0);
            }

            if(gamepad.wasDownPressed()){
                axon.setPosition(1);
            }

            if(gamepad.wasLeftPressed()){
                axon.setPosition(axon.getPosition()-.05);
            }
            if(gamepad.wasLeftPressed()){
                axon.setPosition(axon.getPosition()+.05);
            }


        }
    }
}