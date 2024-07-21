package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "testNewDrive", group = "LinearOpMode")

public class testNewDrive extends LinearOpMode {
    TeamGamepad gamepad;
    Drive drive;
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        drive.initalize();

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.gamepad.right_stick_button && gamepad.gamepad.left_stick_button) {
                drive.setHeading(180);
            }
            drive.universalDriveJoystickV2(
                    gamepad.gamepad.left_stick_y,
                    -gamepad.gamepad.left_stick_x,
                    gamepad.gamepad.right_stick_x,
                    gamepad.gamepad.right_trigger > .5,gamepad.gamepad.left_trigger > .5,
                    drive.getHeading());
        }
    }
}
