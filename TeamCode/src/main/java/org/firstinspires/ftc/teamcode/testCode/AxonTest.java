package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;



@TeleOp(name="Axon Test", group="Linear Opmode")

public class AxonTest extends LinearOpMode {


    public double DEGREES_TO_VOLTAGE(double degrees){
        return (-.00894848)*degrees+3.126;
    }
    public double DEGREES_TO_POSITION(double degrees){
        return degrees/330;
    }
    public double VOLTAGE_TO_POSITION(double voltage){
        return (-voltage/3.126)+1;
    }
    public double VOLTAGE_TO_DEGREES(double voltage){
        return (voltage-3.126)/(-.00894848);
    }
    public double POSITION_TO_VOLTAGE(double position){
        return (position-1)*3.126;
    }
    public double POSITION_TO_DEGREES(double position){
        return position*330;
    }

    public Servo axon;
    TeamGamepad gamepad;
    AnalogInput axonPotentiometer;



    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public void runToPostitionWithStallDetect(double targetDegrees, int timeout){
        axonPotentiometer = hardwareMap.analogInput.get("sensor");
        axon = hardwareMap.servo.get("servo");

        boolean positive;
        if(axon.getPosition()-DEGREES_TO_POSITION(targetDegrees)>0){positive = false;}
        else{positive = true;}

        axon.setPosition(DEGREES_TO_POSITION(targetDegrees));
        double targetVoltage = DEGREES_TO_VOLTAGE(targetDegrees);
        double lastVoltage = axonPotentiometer.getVoltage();
        double changeInVoltage = 0;
        double timeStalled = 0;
        double startStallTime = 0;

        while(Math.abs(axonPotentiometer.getVoltage() - targetVoltage)>.005&&timeStalled<timeout){
            double voltage = axonPotentiometer.getVoltage();

            changeInVoltage = voltage-lastVoltage;

            if(Math.abs(changeInVoltage)<.005){
                if(timeStalled == 0){
                    startStallTime = System.currentTimeMillis();
                    teamUtil.pause(1);
                    teamUtil.log("Start Stall Time" + startStallTime);
                }
                timeStalled = System.currentTimeMillis()-startStallTime;
                teamUtil.log("Time Stalled" + timeStalled);
            }
            else{
                timeStalled = 0;
            }
            lastVoltage = voltage;

        }
        if(timeStalled>timeout){
            teamUtil.log("SERVO STALLED");
            double safePosition = axonPotentiometer.getVoltage()*(-1/3.126)+1;
            if(positive){
                axon.setPosition(safePosition-0.1);
            }
            else{
                axon.setPosition(safePosition+0.1);
            }

        }


    }

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        AnalogInput axonPotentiometer;
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        axonPotentiometer = hardwareMap.analogInput.get("sensor");
        axon = hardwareMap.servo.get("servo");
        axon.setPosition(0.5);
        double lastDegrees;
        double potentiometerDegrees=-1;




        waitForStart();
        while(opModeIsActive()) {
            lastDegrees=potentiometerDegrees;

            double voltage = axonPotentiometer.getVoltage();
            potentiometerDegrees = (voltage-3.126)/(-.00894848);
            telemetry.addLine("Voltage:  " + voltage);
            if(lastDegrees==potentiometerDegrees){
            }else{
                teamUtil.log("Voltage: " + voltage + " Degrees: " + potentiometerDegrees);
                teamUtil.log("Servo Position: " + axon.getPosition());
            }
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

            if(gamepad.wasRightPressed()){
                axon.setPosition(axon.getPosition()+.05);
            }
            if(gamepad.wasHomePressed()){
                runToPostitionWithStallDetect(0, 1000);
            }
            gamepad.loop();

        }
    }



}