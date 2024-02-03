package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.assemblies.PixelRelease;
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

    int currentCam;

    public void toggleCamera() {
        currentCam++;
        if (currentCam > 3) {
            currentCam = 1;
        }
        switch(currentCam) {
            case 1:
                teamUtil.log("Toggling LineFinder On");
                robot.drive.switchCV(Drive.cvCam.FRONT_LINE);
                break;
            case 2:
                teamUtil.log("Toggling AprilTag Finder On");
                robot.drive.switchCV(Drive.cvCam.REAR_APRILTAG);
                ;
                break;
            case 3:
                teamUtil.log("Toggling TeamProp Finder On");
                robot.drive.switchCV(Drive.cvCam.SIDE_PROP);
                break;
            default:
        }
    }

    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        driverGamepad.reset();
        armsGamepad.reset();
        telemetry.addLine("Initializing.  Please wait.");
        telemetry.update();
        robot = new Robot();
        teamUtil.robot = robot;

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
        robot.releaser.initialize();
        robot.drive.initalize(robot.output);

        telemetry.addLine("Ready to start");
        telemetry.update();
        robot.drive.setHeading(180);

        driverGamepad.reset();
        armsGamepad.reset();
        waitForStart();

        driverGamepad.reset();
        armsGamepad.reset();
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
            if(driverGamepad.wasRightTriggerPressed()){ // set drive variables
                armsGamepad.reset();
                driverGamepad.reset();
                while (!driverGamepad.wasRightTriggerPressed() && opModeIsActive()){
                    driverGamepad.loop();
                    if(driverGamepad.wasUpPressed()){
                        robot.a=robot.a+ (driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    }else if(driverGamepad.wasDownPressed()){
                        robot.a=robot.a-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    }else if(driverGamepad.wasLeftPressed()){
                        robot.b=robot.b+(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : .1);
                    } else if(driverGamepad.wasRightPressed()){
                        robot.b=robot.b-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : .1);
                    }
                    if (driverGamepad.wasYPressed()) {
                        robot.c=robot.c+(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : .01);
                    } else if (driverGamepad.wasAPressed()) {
                        robot.c= robot.c-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : .01);
                    } else if (driverGamepad.wasXPressed()) {
                        robot.d=robot.d+(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : .1);
                    }else if (driverGamepad.wasBPressed()) {
                        robot.d=robot.d-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : .1);
                    }
                    telemetry.addLine("Setting Drive Variables");

                    telemetry.addLine("A: " + robot.a);
                    telemetry.addLine("B: " + robot.b);
                    telemetry.addLine("C: " + robot.c);
                    telemetry.addLine("D: " + robot.d);
                    telemetry.update();
                }
                armsGamepad.reset();
                driverGamepad.reset();
            }

            if(driverGamepad.wasLeftPressed()) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.LARSONSCANNERGRAY);
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV4(1, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasUpPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV4(2, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasRightPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV4(3, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasXPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV4(1, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasYPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();

                robot.autoV4(2, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasBPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();

                robot.autoV4(3, useArms, true);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasOptionsPressed()){
                toggleCamera();
            }


            if(driverGamepad.wasAPressed()){
                // Test Something use (a,b,c,d) if you want to
                teamUtil.robot = robot;
                //robot.cycleV4(0,useArms,2,0);

                  //Toggle pixel releaser
                /*
                robot.releaser.toggle();
                if(!robot.releaser.holding){
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);
                }
                else{
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                }

                 */


                // Test April Tag Localization
//                while(!driverGamepad.wasAPressed()){
//                    driverGamepad.loop();
//                    telemetry.addLine("Toggle CV");
//                    telemetry.update();
//                    if(driverGamepad.wasRightBumperPressed()){
//                        robot.drive.switchCV(Drive.cvCam.REAR_APRILTAG);
//                    }
//                }
                int path = (int)robot.a;
                long startTime = System.currentTimeMillis();
                robot.drive.setHeading(180);
                robot.drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                /* from cycle stack */
                //int distance = path==1? 260 : path==2 ? 243: 225;
                int distance = path==1? 275 : path==2 ? 260: 240; // add 15

                robot.driveToBackDropV2(path, useArms,0* (teamUtil.alliance==RED ? 1 : -1),distance+robot.b,3,robot.output.GrabberRotatorHorizontal2, robot.output.StraferLoad);
                if (useArms) {robot.output.dropAndGoToLoad();}
                elapsedTime = System.currentTimeMillis()-startTime;
                teamUtil.log("Elapsed Time Path "+ path+" : " + ((float)(System.currentTimeMillis()-startTime)/(float)1000));

//
//                /* from wing stack */
//                robot.drive.moveCm(robot.drive.MAX_VELOCITY,2,0,180,750);
//                robot.intake.ready();
//                robot.drive.moveCm(robot.drive.MAX_VELOCITY,15,0,180,750);
//                int distance = path==1? 235 : path==2 ? 220: 200;
//                double rotation, strafe;
//                if(path==2||path==3){
//                    rotation = robot.output.GrabberRotatorHorizontal2;
//                    strafe = robot.output.StraferLoad+4*robot.output.StraferPositionPerCm;
//                }else{
//                    rotation = robot.output.GrabberRotatorHorizontal1;
//                    strafe = robot.output.StraferLoad-4*robot.output.StraferPositionPerCm;
//                }
//                if(teamUtil.alliance == RED) {
//                    robot.drive.strafeToEncoder(90, 180, 1000, 3200+robot.c, 2000); //strafe value was 16700 when res //TODO adjust strafe for off wall
//                } else {
//                    robot.drive.strafeToEncoder(270,180,1000,-2250-robot.d,2000); //strafe value was 15750 when res//TODO adjust strafe for off wall
//                }
//                robot.driveToBackDrop(path, useArms,4000* (teamUtil.alliance==RED ? 1 : -1),distance+robot.b,3,rotation, strafe); //was 17500//TODO adjust strafe for off wall
//
//
//                if (useArms) {robot.output.dropAndGoToLoad();}
//                elapsedTime = System.currentTimeMillis()-startTime;
//                teamUtil.log("Elapsed Time Path "+ path+" : " + ((float)(System.currentTimeMillis()-startTime)/(float)1000));


                  // Test pushPurplePixelWing
            /*
                  while(!driverGamepad.wasAPressed()){
                      driverGamepad.loop();
                      telemetry.addLine("Ready to Toggle");
                      telemetry.update();
                      if(driverGamepad.wasRightBumperPressed()){
                          robot.releaser.toggle();
                      }if(driverGamepad.wasLeftBumperPressed()){
                          robot.intake.ready();
                      }if(driverGamepad.wasRightTriggerPressed()){
                          robot.intake.collectTopPixel();
                      }
                      if(driverGamepad.wasOptionsPressed()){
                          toggleCamera();
                      }
                  }

                  long startTime = System.currentTimeMillis();
                  robot.drive.setHeading(180);
                  /*
                  robot.autoV4((int)robot.a,useArms,true);
                  //robot.pushPurplePlaceYellowPixelWingV4(1+(int)robot.d,useArms);
                  teamUtil.log("Elapsed Time: " + ((float)(System.currentTimeMillis()-startTime)/1000f));

                   */



                /* Test Cycle V4
                  robot.drive.setHeading(180);
                  long startTime = System.currentTimeMillis();
                  robot.cycleV4(-robot.drive.TAG_CENTER_TO_CENTER,true,2);
                  teamUtil.log("Elapsed Time: " + (float)((System.currentTimeMillis()-startTime)/1000));
                 */
                /* Test auto Grab Two
                  robot.intake.ready();
                  robot.intake.startIntake();
                  teamUtil.pause(2000);
                  robot.intake.autoGrabTwoNoWait();
                 */
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            }




            telemetry.addLine("Running Tests " );
            telemetry.addLine("Last Auto Elapsed Time: " + elapsedTime);
            telemetry.addLine("A: " + robot.a);
            telemetry.addLine("B: " + robot.b);
            telemetry.addLine("C: " + robot.c);
            telemetry.addLine("D: " + robot.d);
            telemetry.update();
        }
    }
}
