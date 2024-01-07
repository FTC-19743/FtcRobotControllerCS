package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;


import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;

    Blinkin blinkin;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;

    /*
    public void loopRunTimeCalculate(int loopNumber,boolean button){
        long startTime=0;
        long endTime=0;
        int loopAmount=0;
        int buttonPressNumber=0;
        if(button&&buttonPressNumber==0){
            buttonPressNumber=1;
            startTime=System.currentTimeMillis();
        }
        if(button&&buttonPressNumber==1){
            loopAmount=loopNumber;
            endTime=System.currentTimeMillis();
        }
        long totalRunTime = endTime-startTime;
        long loopTime = totalRunTime/loopAmount;

        //TODO: take away (only for testing)
        telemetry.addLine("Button Press Number" + buttonPressNumber);

        teamUtil.log("Loop Time" + loopTime);


    }

     */


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
        if(!teamUtil.finishedGoToLoad){
            robot.output.goToLoad();

        }
        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
        telemetry.update();
        double manualSpeedElevator = 100;
        robot.output.lastLevel = 2;
        robot.drive.setHeading(180);
        while (!opModeIsActive()) {
            driverGamepad.loop();
            if(driverGamepad.wasRightBumperPressed()||driverGamepad.wasLeftBumperPressed()){
                if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }else{
                    teamUtil.alliance= teamUtil.Alliance.BLUE;
                }
            }
            telemetry.addLine("Ready to start");
            telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
            telemetry.update();
        }
        int loopRunNumber=0;
        waitForStart();


        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();

            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(180);
            }
            /*
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

             */

            if (teamUtil.alliance == teamUtil.Alliance.RED) { // Make this work for Red and Blue Alliances
                robot.drive.universalDriveJoystickV2(
                        driverGamepad.gamepad.left_stick_y,
                        -driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,driverGamepad.gamepad.left_trigger > .5,
                        robot.drive.getHeading());
            } else {
                robot.drive.universalDriveJoystickV2(
                        -driverGamepad.gamepad.left_stick_y,
                        driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5, driverGamepad.gamepad.left_trigger > .5,
                        robot.drive.getHeading());
            }

            //if(driverGamepad.wasAPressed()){
                //robot.drive.setHeldHeading(robot.driverSide());
            //}
            if(driverGamepad.wasYPressed()||driverGamepad.wasAPressed()){
                robot.drive.setHeldHeading(robot.fieldSide());
            }
            if(driverGamepad.wasXPressed()||driverGamepad.wasBPressed()){
                robot.drive.setHeldHeading(180);
            }
            //if(driverGamepad.wasBPressed()){
                //robot.drive.setHeldHeading(teamUtil.alliance== teamUtil.Alliance.RED?0:180);
            //}

            ////////// Intake
            robot.intake.autoOff();


            if(armsGamepad.wasLeftTriggerPressed()){
                robot.intake.toggleIntake();
            }

            if(armsGamepad.wasRightPressed()){
                //Needs to be implemented; free for now
                robot.intake.grabTwoPixels();
            }

            if(gamepad2.dpad_left){
                robot.intake.grabOnePixelLoop(gamepad2.dpad_left);

            }
            if(armsGamepad.wasDownPressed()){
                robot.intake.store();
            }

            if(armsGamepad.wasUpPressed()){
                robot.intake.ready();
            }



            ////////// Lift
            if(driverGamepad.gamepad.left_bumper && driverGamepad.gamepad.right_bumper){
                robot.lift.raiseLift();
            } else if (robot.lift.startedLifting){
                robot.lift.holdLift();
            }
            if (driverGamepad.wasUpPressed()) {
                robot.lift.toggleArm();
            }
            if (false) {
                robot.lift.stowArm();
            }

            ///////// Drone Launcher

            if(driverGamepad.wasOptionsPressed()){
                robot.launcher.toggleRelease();
            }

            ////////// Output
            if(armsGamepad.wasAPressed()){ // Get ready to load the next pixels
                robot.output.goToLoadNoWait();
            }
            if(armsGamepad.wasYPressed()){ // Send output system to scoring position
                //robot.output.goToScoreNoWait(3);
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                robot.output.goToScoreNoWait(robot.output.lastLevel,robot.output.GrabberRotatorLoad);
            }

            if(gamepad2.left_bumper||gamepad2.right_bumper){
                robot.output.straferManualV2(gamepad2.left_bumper,gamepad2.right_bumper);
                //robot.output.straferManual(true);
            }
            /*
            if(armsGamepad.wasRightBumperPressed()){
                robot.output.straferManual(false);
            }

             */

            if(Math.abs(armsGamepad.gamepad.left_stick_y) > .30){
                //robot.output.elevManual(-(armsGamepad.gamepad.left_stick_y)*manualSpeedElevator);
                robot.output.elevManualV2(-(armsGamepad.gamepad.left_stick_y));
            }
            if (armsGamepad.gamepad.right_trigger> 0.5) {
                robot.output.dropAndGoToLoadNoWait();
            }


            if(armsGamepad.wasRightJoystickFlickedLeft()){
                robot.output.rotateGrabberCounterclockwise();
            }
            if(armsGamepad.wasRightJoystickFlickedRight()){
                robot.output.rotateGrabberClockwise();
            }

            if(armsGamepad.wasStartPressed()){
                robot.intake.reverseIntake();
            }

            /*
            if(armsGamepad.wasOptionsPressed()) {
                loopRunTimeCalculate(loopRunNumber,armsGamepad.wasOptionsPressed());
            }

             */


            //loopRunNumber++;
            robot.outputTelemetry();
            //robot.telemetry.addLine("Loop Run Number: " + loopRunNumber);
            robot.telemetry.addLine("Last Level: " + robot.output.lastLevel);
            telemetry.update();

        }
    }
}
