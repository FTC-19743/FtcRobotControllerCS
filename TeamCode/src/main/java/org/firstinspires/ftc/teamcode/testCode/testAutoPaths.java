package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "testAutoPaths", group = "LinearOpMode")
public class testAutoPaths extends LinearOpMode {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot robot;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean useArms = false;
    boolean liveStream = true;

    public long startTime;
    public long elapsedTime;

    private boolean enableLiveView = false;

    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        telemetry.addLine("Initializing.  Please wait.");
        telemetry.update();
        robot = new Robot();


        while(!driverGamepad.wasAPressed()){
            driverGamepad.loop();
            if(driverGamepad.wasLeftPressed()){ enableLiveView = !enableLiveView;}
            if(driverGamepad.wasRightPressed()){ enableLiveView = !enableLiveView;}

            teamUtil.telemetry.addLine("LiveView Enabled? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine("LiveView: " + enableLiveView);
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }
        telemetry.addLine("Initializing CV.  Please wait.");
        telemetry.update();
        robot.drive.initCV(enableLiveView);
        robot.intake.initalize();
        robot.output.initialize();
        robot.output.calibrate();
        robot.drive.initalize(robot.output);

        telemetry.addLine("Ready to start");
        telemetry.update();
        robot.drive.setHeading(180);
        waitForStart();


        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();
            telemetry.addLine("Alliance: "+ teamUtil.alliance);
            telemetry.addLine("Use Arms: "+ useArms);
            telemetry.addLine("Strafe: "+ robot.drive.strafeEncoder.getCurrentPosition());
            telemetry.addLine("Rear Vision Portal FPS: "+ robot.drive.rearVisionPortal.getFps());


            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(180);
            }
            if (teamUtil.alliance == teamUtil.Alliance.RED) {
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

            if (driverGamepad.wasLeftBumperPressed()) {
                if (teamUtil.alliance== teamUtil.Alliance.RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }
            }
            if (driverGamepad.wasRightBumperPressed()) {
                useArms = !useArms;
            }
            if (driverGamepad.wasLeftTriggerPressed()) {
                if (liveStream) {
                    liveStream = false;
                    robot.drive.sideVisionPortal.stopLiveView();
                } else {
                    liveStream = true;
                    robot.drive.sideVisionPortal.resumeLiveView();
                }
            }

            if(driverGamepad.wasLeftPressed()) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LARSONSCANNERGRAY);
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV3(1, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasUpPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV3(2, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasRightPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV3(3, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasXPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV3(1, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasYPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();

                robot.autoV3(2, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasBPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();

                robot.autoV3(3, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasOptionsPressed()){
                robot.drive.findMaxVelocity(100);
            }
            if(driverGamepad.wasAPressed()){
                robot.drive.findMaxStrafeVelocity(100);

            }
            //test for cycle pathing
//            if(driverGamepad.wasDownPressed()){
//                robot.drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OCEANPALETTE);
//                robot.drive.setHeading(180);
//                //robot.drive.moveStraightCmWithStrafeEncoder(robot.drive.MAX_VELOCITY, 125, 0,0, 180, robot.drive.MAX_VELOCITY);
//                robot.drive.moveStraightCmWithStrafeEncoderWithGoToScore(robot.drive.MAX_VELOCITY, 215, 0,130,0, 180, 600);
//                //robot.drive.moveStraightCmWithStrafeEncoder(robot.drive.MAX_VELOCITY, 91, 0,0, 180, 1000);
//                robot.drive.moveCm(robot.drive.MAX_VELOCITY, 80, 315, 180, 0);
//                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
//                robot.output.dropAndGoToLoad();
//
//            }
            telemetry.addLine("Last Auto Elapsed Time: " + elapsedTime);
            telemetry.update();

        }
    }
}
