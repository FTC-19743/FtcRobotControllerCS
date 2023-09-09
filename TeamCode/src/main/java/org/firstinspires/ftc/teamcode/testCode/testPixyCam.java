package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.PixyBlock;
import org.firstinspires.ftc.teamcode.libs.PixyCam2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "testPixyCam")
@Disabled
public class testPixyCam extends LinearOpMode {
    TeamGamepad gamepad;
    private Servo servo;


    public void runOpMode() {
        PixyCam2 pixyCam = hardwareMap.get(PixyCam2.class, "pixycam");
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if (gamepad.wasAPressed()) {
                pixyCam.setLEDs((byte) 255, (byte) 0, (byte) 0);
            }
            if (gamepad.wasBPressed()) {
                pixyCam.setLEDs((byte) 0, (byte) 255, (byte) 0);
            }
            if (gamepad.wasXPressed()) {
                pixyCam.setLEDs((byte) 0, (byte) 0, (byte) 255);
            }
            if (gamepad.wasUpPressed()) {
                pixyCam.getVersionInfo();
            }
            if (gamepad.wasLeftPressed()) {
                pixyCam.toggleLEDs(true, true);
            }
            if (gamepad.wasRightPressed()) {
                pixyCam.toggleLEDs(false, false);
            }
            while (gamepad1.dpad_down) {
                PixyBlock[] blocks = pixyCam.getBlocks((byte) 0b11111111, (byte) 3);
                //TODO change request for one block and check for returned blocks
                if (blocks.length == 0){
                    teamUtil.log("Nothing was detected");
                }else {
                    teamUtil.log("X Center: " + blocks[0].xCenter);
                    teamUtil.log("Y Center: " + blocks[0].yCenter);
                    teamUtil.log("Width: " + blocks[0].width);
                    teamUtil.log("Height: " + blocks[0].height);
                }
            }

            //pixyCam.showBug();
            telemetry.addLine(
                    "PixyCam: HW:" + pixyCam.getHWVersion() +
                            " FW:" + pixyCam.getFWVersionMajor() +
                            "." + pixyCam.getFWVersionMinor() +
                            " Build:" + pixyCam.getBuildNum() +
                            " Type:" + pixyCam.getFWType());
            telemetry.update();
        }


// 2022-04-27 07:13:22.441  1684-2383  I2C                     com.qualcomm.ftcrobotcontroller      I  Automatically initializing I2C device PixyCam2 USB (embedded); module 2; bus 1; addr7=0x18
    }
}