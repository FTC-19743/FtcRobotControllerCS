package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        if (!teamUtil.justRanAuto) { // Auto already took care of this, so save time.
            robot.calibrate();
        }
        telemetry.addLine("Ready to start");
        telemetry.update();
        double manualSpeedElevator = 100;
        robot.output.lastLevel = 2;
        robot.drive.setHeading(180);
        waitForStart();


        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();

            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(180);
            }
            if (teamUtil.alliance == teamUtil.Alliance.RED) { // Make this work for Red and Blue Alliances
                robot.drive.universalDriveJoystick(
                        driverGamepad.gamepad.left_stick_y,
                        -driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,
                        robot.drive.getHeading());
            } else {
                robot.drive.universalDriveJoystick(
                        -driverGamepad.gamepad.left_stick_y,
                        driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,
                        robot.drive.getHeading());
            }

            if(driverGamepad.wasDownPressed()){
                robot.drive.driveToTapeTelopNoWait(0,180,400,3000);
            }

            ////////// Intake
            robot.intake.autoOff();


            if(armsGamepad.wasLeftTriggerPressed()){
                robot.intake.toggleIntake();
            }

            if(armsGamepad.wasUpPressed()){
                robot.intake.collect();
            }
            if (armsGamepad.wasLeftPressed())
            {
                robot.intake.grabOnePixel();

            }
            if(armsGamepad.wasDownPressed()){
                robot.intake.store();
            }
            /*
            if(driverGamepad.wasDownPressed()){
                robot.auto(1,false);
            }


             */


            ////////// Lift
            if(driverGamepad.gamepad.left_bumper && driverGamepad.gamepad.right_bumper){
                robot.lift.raiseLift();
            } else if (robot.lift.startedLifting){
                robot.lift.holdLift();
            }
            if (driverGamepad.wasBPressed()) {
                robot.lift.toggleArm();
            }
            if (false) {
                robot.lift.stowArm();
            }

            ///////// Launcher

            if(driverGamepad.wasOptionsPressed()){
                robot.launcher.toggleRelease();
            }

            ////////// Output
            if(armsGamepad.wasAPressed()){ // Get ready to load the next pixels
                robot.output.goToLoadNoWait();
            }
            if(armsGamepad.wasYPressed()){ // Send output system to scoring position
                //robot.output.goToScoreNoWait(3);
                robot.output.goToScoreNoWait(robot.output.lastLevel);
            }
            /*
            if(armsGamepad.wasLeftBumperPressed()){
                robot.output.straferManual(true);
            }

            if(armsGamepad.wasRightBumperPressed()){
                robot.output.straferManual(false);
            }

             */

            if(Math.abs(armsGamepad.gamepad.left_stick_y) > .30){
                robot.output.elevManual(-(armsGamepad.gamepad.left_stick_y)*manualSpeedElevator);
            }
            if (armsGamepad.gamepad.right_trigger> 0.5) {
                robot.output.dropAndGoToLoadNoWait();
            }

            if(armsGamepad.wasXPressed()){
                robot.output.rotateGrabberCounterclockwise();
            }
            if(armsGamepad.wasBPressed()){
                robot.output.rotateGrabberClockwise();
            }

            if(armsGamepad.wasStartPressed()){
                robot.intake.reverseIntake();
            }

            robot.outputTelemetry();
            robot.telemetry.addLine("Last Level: " + robot.output.lastLevel);
            telemetry.update();

        }
    }
}
