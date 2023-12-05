package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
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


    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        robot = new Robot();
        robot.drive.initalize();
        robot.drive.initCV();
        robot.intake.initalize();
        robot.output.initialize();
        robot.output.calibrate();
        while(!driverGamepad.wasAPressed()){
            driverGamepad.loop();
            if(driverGamepad.wasLeftPressed()){ teamUtil.alliance = teamUtil.Alliance.RED;}
            if(driverGamepad.wasRightPressed()){ teamUtil.alliance = teamUtil.Alliance.BLUE;}

            teamUtil.telemetry.addLine("RED or BLUE? (use Game Pad 1 DPad)");
            teamUtil.telemetry.addLine(teamUtil.alliance == teamUtil.Alliance.RED ? "RED Alliance" : "BLUE Alliance");
            teamUtil.telemetry.addLine("------------------------------------");
            teamUtil.telemetry.addLine("Then press A on Game Pad 1 to move on");
            teamUtil.telemetry.update();
        }

        telemetry.addLine("Ready to start");
        telemetry.update();
        robot.drive.setHeading(180);
        waitForStart();


        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();

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
            if(driverGamepad.wasLeftPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.autoV3(1, false);
            }
            if(driverGamepad.wasUpPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.autoV3(2, true);
            }
            if(driverGamepad.wasRightPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.autoV3(3, false);
            }
            if(driverGamepad.wasXPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.autoV3(1, true);
            }
            if(driverGamepad.wasYPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.autoV3(2, true);
            }
            if(driverGamepad.wasBPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.autoV3(3, true);
            }

            telemetry.update();

        }
    }
}
