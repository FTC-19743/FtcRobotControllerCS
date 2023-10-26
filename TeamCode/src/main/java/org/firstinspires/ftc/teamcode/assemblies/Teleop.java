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
        double manualSpeedElevator = 100;
        double manualSpeedStrafer = 0.01;

        waitForStart();


        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();

            ////////// Drive
            robot.drive.universalDriveJoystick(
                    driverGamepad.gamepad.left_stick_x,
                    driverGamepad.gamepad.left_stick_y,
                    driverGamepad.gamepad.right_stick_x,
                    driverGamepad.gamepad.right_trigger>.5,
                    robot.drive.getHeading());

            ////////// Intake
            if(gamepad1.a){
                robot.intake.startIntake();
            }
            if(gamepad1.x){
                robot.intake.reverseIntake();
            }
            if(gamepad1.b){
                robot.intake.stopIntake();
            }

            ////////// Lift
            if(gamepad1.left_bumper&&gamepad1.right_bumper){
                robot.output.moveLift();
            }
            // TODO: Control for Lift Arm

            ////////// Output
            if(armsGamepad.wasDownPressed()){
                robot.output.goToLoadNoWait();
            }
            // TODO: Need a grabAndDeployNoWait method here to grab the pixels and get the output system ready to place

            if(Math.abs(gamepad2.left_stick_x) > .10){
                robot.output.straferManual(-(gamepad2.left_stick_x)*manualSpeedStrafer);
            }
            if(armsGamepad.wasBPressed()){
                robot.output.dropPixels();
            }
            if(armsGamepad.wasXPressed()){
                robot.output.grabPixels();
            }

            if(Math.abs(gamepad2.left_stick_y) > .10){
                robot.output.elevManual(-(gamepad2.left_stick_y)*manualSpeedElevator);
            }

            robot.outputTelemetry();
            telemetry.update();

        }
    }
}
