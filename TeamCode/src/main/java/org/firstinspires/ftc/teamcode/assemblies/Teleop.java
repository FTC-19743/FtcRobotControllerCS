package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot robot;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;


    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        robot = new Robot();
        robot.initialize();
        robot.calibrate();
        telemetry.addLine("Ready to start");
        telemetry.update();
        double powerFactor = 1;

        waitForStart();
        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();



            if(gamepad1.a){
                robot.intake.startIntake();
            }

            if(gamepad1.x){
                robot.intake.reverseIntake();
            }
            if(gamepad1.b){
                robot.intake.stopIntake();

            }

            robot.output.manualLiftChange(gamepad2.left_stick_y*1000);

            if(armsGamepad.wasUpPressed()){
                robot.output.grabberRotatorAddManualIncrement(0.01);
            }

            if(armsGamepad.wasDownPressed()){
                robot.output.grabberRotatorSubtractManualIncrement(0.01);
            }

            if(armsGamepad.wasLeftPressed()){
                robot.output.straferAddManualIncrement(0.01);
            }

            if(armsGamepad.wasRightPressed()){
                robot.output.straferSubtractManualIncrement(0.01);
            }

            if(armsGamepad.wasBPressed()){
                robot.output.dropPixels();
            }
            if(armsGamepad.wasXPressed()){
                robot.output.dropPixels();
            }
            /*

            //declaration of power and denominator variables for math
            double flPower;
            double frPower;
            double blPower;
            double brPower;
            double denominator;

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x*1.1 ; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            /*
            if(Math.abs(rx)<0.15){
                rx=0;
            }

             /*
            //Orientation anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double botHeading = -Math.toRadians((robot.drive.getHeading()-180)); //
            //double botHeading = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle; //removed negative
            telemetry.addLine("botheading: " + Math.toDegrees(botHeading));
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            //telemetry.addLine("current cup level "+ String.valueOf(cupLevel));

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]





            //working field centric
            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            flPower = (rotY + rotX + rx) / denominator;
            blPower = (rotY - rotX + rx) / denominator;
            frPower = (rotY - rotX - rx) / denominator;
            brPower = (rotY + rotX - rx) / denominator;






            robot.drive.fl.setPower(-flPower*powerFactor);
            robot.drive.bl.setPower(-blPower*powerFactor);
            robot.drive.fr.setPower(-frPower*powerFactor);
            robot.drive.br.setPower(-brPower*powerFactor);

             */


            robot.outputTelemetry();
            telemetry.update();

        }
    }
}
