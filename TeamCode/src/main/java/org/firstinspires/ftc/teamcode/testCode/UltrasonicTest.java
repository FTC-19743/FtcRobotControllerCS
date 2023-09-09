package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name="Ultrasonic Test", group="Linear Opmode")
@Disabled
public class UltrasonicTest extends LinearOpMode {
    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        double CMS_PER_INCH = 2.54;
        AnalogInput ultrasonic;
        ultrasonic = hardwareMap.analogInput.get("ult");
        waitForStart();
        while(opModeIsActive()) {
            double voltage = ultrasonic.getVoltage();
            double distance = (260/3*(voltage-.55)+36)*2.54;
            telemetry.addLine("voltage:  " + Double.toString(voltage));
            telemetry.addLine("distance:  "+ Double.toString(distance));
            telemetry.update();
            //log(""+distance);


        }
    }
}
