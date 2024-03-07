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
    boolean endgame = false;

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
    //tentative test code
    public void coordinatePositionOnFieldTelemetry(){
        double xPosition;
        double yPosition;
        if(teamUtil.alliance == teamUtil.Alliance.RED&&teamUtil.SIDE == teamUtil.Side.WING){
            xPosition=132+((double)(robot.drive.strafeEncoder.getCurrentPosition()/130)/2.54);
            yPosition = 36+((double)(robot.drive.forwardEncoder.getCurrentPosition()/775)/2.54);
            //132,36
        }else if(teamUtil.alliance == teamUtil.Alliance.RED&&teamUtil.SIDE == teamUtil.Side.SCORE){
            xPosition=132+((double)(robot.drive.strafeEncoder.getCurrentPosition()/130)/2.54);
            yPosition = 86+((double)(robot.drive.forwardEncoder.getCurrentPosition()/775)/2.54);
            //132,84
        }else if(teamUtil.alliance == teamUtil.Alliance.BLUE&&teamUtil.SIDE == teamUtil.Side.WING){
            xPosition=12+((double)(robot.drive.strafeEncoder.getCurrentPosition()/130)/2.54);
            yPosition = 36+((double)(robot.drive.forwardEncoder.getCurrentPosition()/775)/2.54);
            //12,36
        }else{
            xPosition=12+((double)(robot.drive.strafeEncoder.getCurrentPosition()/130)/2.54);
            yPosition = 84+((double)(robot.drive.forwardEncoder.getCurrentPosition()/775)/2.54);
            //12,84
        }
        telemetry.addLine("Current X: " + xPosition + " Inches" + "Y:" + yPosition+" Inches");
    }


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
        robot.intake.startIntake();


        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();

            if(driverGamepad.wasHomePressed()){
                endgame=true;
            }

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
            robot.intake.autoOffV2();

            if(gamepad2.start){
                robot.intake.stopIntake();
            }

            if(armsGamepad.wasLeftTriggerPressed()){
                robot.intake.ready();
            }

            if(gamepad2.back){
                robot.intake.ready();
                robot.intake.reverseIntake();
            }

            if(armsGamepad.wasBackPressed()){
                robot.intake.startIntake();
            }


            if(armsGamepad.wasRightPressed()||armsGamepad.wasLeftPressed()){
                robot.intake.teleopFlickOneNoWait();
            }
            if(gamepad2.right_trigger>0.5&&robot.output.loading.get()){
                robot.intake.resetFlicker();
            }
            if(armsGamepad.wasDownPressed()){
                robot.intake.teleopGetOneNoWait();
            }

            if(armsGamepad.wasUpPressed()){
                robot.intake.teleopGetTwoNoWait();
            }



            ////////// Lift
            if(driverGamepad.gamepad.left_bumper && driverGamepad.gamepad.right_bumper&&endgame){
                robot.lift.raiseLift();
            } else if (robot.lift.startedLifting&&endgame){
                robot.lift.lowerLift();
            }
            if (driverGamepad.wasUpPressed()) {
                if(endgame){
                    robot.lift.toggleArm();
                }

            }
            if (false&&endgame) {
                robot.lift.stowArm();
            }

            ///////// Drone Launcher

            if(driverGamepad.wasOptionsPressed()){
                if(endgame){
                    robot.launcher.toggleRelease();
                }

            }

            ////////// Output
            if(armsGamepad.wasAPressed()){ // Go to score with only one pixel
                if(robot.intake.pixelsLoaded == 1){
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                    robot.output.goToScoreNoWait(robot.output.lastLevel,robot.output.GrabberRotatorLoad,robot.output.StraferLoad);
                }

            }
            if(armsGamepad.wasYPressed()){ // Send output system to scoring position
                if(robot.intake.pixelsLoaded == 2){
                    if(!robot.output.loading.get()&&!robot.output.moving.get()) {
                        robot.output.grabberRotater.setPosition(robot.output.GrabberRotatorLoad);
                    }else{
                        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                        robot.output.goToScoreNoWait(robot.output.lastLevel, robot.output.GrabberRotatorLoad, robot.output.StraferLoad);
                    }
                }

            }

            if(armsGamepad.wasXPressed()){ // Send output system to scoring position
                if(robot.intake.pixelsLoaded == 2){
                    //robot.output.goToScoreNoWait(3);
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                    //robot.output.goToScoreNoWait(robot.output.lastLevel,robot.output.GrabberRotatorLoad + robot.output.GrabberRotatorIncrement/2,robot.output.StraferLoad);
                    robot.output.goToScoreNoWait(robot.output.lastLevel,robot.output.rotatorPos[4],robot.output.StraferLoad);
                    robot.output.currentRotatorPos = 4;
                }

            }

            if(armsGamepad.wasBPressed()){ // Send output system to scoring position
                if(robot.intake.pixelsLoaded == 2){
                    //robot.output.goToScoreNoWait(3);
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                    robot.output.goToScoreNoWait(robot.output.lastLevel,robot.output.rotatorPos[2],robot.output.StraferLoad);
                    robot.output.currentRotatorPos = 2;
                    //robot.output.goToScoreNoWait(robot.output.lastLevel,robot.output.GrabberRotatorLoad - robot.output.GrabberRotatorIncrement/2,robot.output.StraferLoad);
                }
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
            if (armsGamepad.gamepad.right_trigger> 0.5&&!robot.output.loading.get()) {
                robot.output.dropAndGoToLoadNoWait();
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            }


            if(armsGamepad.wasRightJoystickFlickedLeft()){
                robot.output.rotateGrabberCounterclockwise();
            }
            if(armsGamepad.wasRightJoystickFlickedRight()){
                robot.output.rotateGrabberClockwise();
            }

            if(armsGamepad.wasStartPressed()){

            }

            /*
            if(armsGamepad.wasOptionsPressed()) {
                loopRunTimeCalculate(loopRunNumber,armsGamepad.wasOptionsPressed());
            }

             */


            //loopRunNumber++;
            robot.outputTelemetry();
            telemetry.addLine("# Pixels present:  " + robot.intake.pixelsLoaded);
            //robot.telemetry.addLine("Loop Run Number: " + loopRunNumber);
            robot.telemetry.addLine("Last Level: " + robot.output.lastLevel);
            if(endgame){
                robot.telemetry.addLine("ENDGAME!!!!!!!");
            }else{
                robot.telemetry.addLine("Not Endgame");
            }


            telemetry.update();


        }
    }
}
