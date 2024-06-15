package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
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
    boolean cycle = true;

    boolean proximity = false;

    public long startTime;
    public long elapsedTime;



    private boolean enableLiveView = false;

    int currentCam = 3;

    public void toggleCamera() {
        currentCam++;
        if (currentCam > 4) {
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
            case 4:
                teamUtil.log("Toggling StackFinder On");
                robot.drive.switchCV(Drive.cvCam.PIXEL_STACK);
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
            telemetry.addLine("Cycle: "+ cycle);

            telemetry.addLine("Strafe: "+ robot.drive.strafeEncoder.getCurrentPosition());
            telemetry.addLine("Forward: "+ robot.drive.forwardEncoder.getCurrentPosition());
            telemetry.addLine("Rear Vision Portal FPS: "+ robot.drive.rearVisionPortal.getFps());


            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(180);
                robot.drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (teamUtil.alliance == RED) {
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
                if (teamUtil.alliance== RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = RED;
                }
            }
            if (driverGamepad.wasRightBumperPressed()) {
                useArms = !useArms;
            }
            if (driverGamepad.wasHomePressed()) {
                proximity = !proximity;
            }
            if (driverGamepad.wasLeftTriggerPressed()) {
                robot.intake.stopIntake();
                cycle = !cycle;

                /*
                if (liveStream) {
                    liveStream = false;
                    robot.drive.sideVisionPortal.stopLiveView();
                } else {
                    liveStream = true;
                    robot.drive.sideVisionPortal.resumeLiveView();
                }

                 */
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
                        robot.b=robot.b+(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    } else if(driverGamepad.wasRightPressed()){
                        robot.b=robot.b-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    }
                    if (driverGamepad.wasYPressed()) {
                        robot.c=robot.c+(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    } else if (driverGamepad.wasAPressed()) {
                        robot.c= robot.c-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    } else if (driverGamepad.wasXPressed()) {
                        robot.d=robot.d+(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
                    }else if (driverGamepad.wasBPressed()) {
                        robot.d=robot.d-(driverGamepad.gamepad.left_bumper ? 10 : driverGamepad.gamepad.left_trigger > .5 ? 100 : 1);
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
                robot.autoV5(1, useArms, 0,cycle,proximity);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasUpPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV5(2, useArms, 0,cycle,proximity);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasRightPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV5(3, useArms, 0,cycle,proximity);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasXPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.autoV5(1, useArms, 0,cycle,proximity);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasYPressed()) {
                teamUtil.SIDE=teamUtil.Side.WING;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();

                robot.autoV5(2, useArms, 0,cycle,proximity);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasBPressed()) {
                teamUtil.SIDE=teamUtil.Side.SCORE;
                robot.drive.setHeading(180);
                startTime=System.currentTimeMillis();
                robot.pushPurplePlaceYellowPixelScoreV5((int) robot.a, false);
                elapsedTime=System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasOptionsPressed()){
                toggleCamera();
            }

            if(driverGamepad.wasStartPressed()){
                //robot.intake.putFlickerDown();
            }


            if(driverGamepad.wasAPressed()){
                teamUtil.robot = robot;
                startTime=System.currentTimeMillis();
                robot.drive.setHeading(180);
                int path = (int)robot.a;
                double xOffset = path == 2 ? 0 : (path == 1 ? -robot.drive.TAG_CENTER_TO_CENTER : robot.drive.TAG_CENTER_TO_CENTER);
                if(robot.pushPurplePlaceYellowPixelScoreV6(2,useArms)){
                    if(robot.insideCycle(0,useArms,1)){
                        robot.insideCycle(0,useArms,2);
                    }
                }
                elapsedTime=System.currentTimeMillis()-startTime;
                teamUtil.log("Elapsed Time: "+elapsedTime);


                //robot.insideCycle(xOffset,false,1);

                /*
                while(!driverGamepad.wasAPressed()){
                    driverGamepad.loop();


                    telemetry.addLine("Toggle Releaser With Right Bumper");
                    telemetry.addLine("Toggle Lid With Left Bumper");

                    telemetry.update();
                    if(driverGamepad.wasRightBumperPressed()){
                        robot.releaser.toggle();
                    }
                    if(driverGamepad.wasLeftBumperPressed()){
                        robot.intake.openLid();
                    }
                }
                /*
                // Test Something use (a,b,c,d) if you want to
                teamUtil.robot = robot;
                robot.drive.setHeading(180);
                int cycleNumber=0;

                while(!driverGamepad.wasAPressed()){
                    driverGamepad.loop();
                    if(driverGamepad.wasUpPressed()){
                        cycleNumber++;
                    }else if(driverGamepad.wasDownPressed()){
                        cycleNumber--;
                    }
                    telemetry.addLine("Cycle Number: " + cycleNumber);

                    telemetry.addLine("Toggle Releaser With Right Bumper");
                    telemetry.addLine("Toggle Lid With Left Bumper");

                    telemetry.update();
                    if(driverGamepad.wasRightBumperPressed()){
                        robot.releaser.toggle();
                    }
                    if(driverGamepad.wasLeftBumperPressed()){
                        robot.intake.openLid();
                    }
                }

                robot.pushPurplePlaceYellowPixelWingV5((int)robot.a,useArms);
                double xOffset = robot.a == 2 ? 0 : (robot.a == 1 ? -robot.drive.TAG_CENTER_TO_CENTER : robot.drive.TAG_CENTER_TO_CENTER);
                for (int i = 0; i < cycleNumber; i++) {
                    if(i==0){
                        robot.cycleV5(xOffset,useArms,(int)robot.a,0);
                    }else{
                        robot.cycleV5(teamUtil.alliance==teamUtil.alliance.RED? -robot.drive.TAG_CENTER_TO_CENTER :robot.drive.TAG_CENTER_TO_CENTER,useArms,(int)robot.a,0);
                    }
                }

                 */
                //robot.pushPurplePlaceYellowPixelWingV5((int)robot.a,useArms);



//                robot.drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                //robot.drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                startTime=System.currentTimeMillis();
//                //robot.pushPurplePlaceYellowPixelWingV5((int)robot.a,useArms);
//                robot.cycleV6(0,useArms,1,System.currentTimeMillis());
//                //robot.cycleV6(0,useArms,2,System.currentTimeMillis());
//
//                //robot.driveToBackDropInsideFastEncodersOnly(useArms,0);
//                elapsedTime=System.currentTimeMillis()-startTime;
//                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);


                // Test drive to stack
                //robot.drive.setHeading(180);
                //robot.drive.driveStraightToTargetWithStrafeEncoderValue(2000, -40000, 0, 180, 180, 1000+robot.a, 5000);
                //robot.drive.driveStraightToTarget(2000, -40000, 180, 180, 1000, 5000);
                //robot.drive.driveToStackNoStopWithStrafeV3(180, 180, 1000, 0, true,3500);
                //robot.drive.stopMotors();






                //robot.drive.driveStraightToTargetWithStrafeEncoderValue(robot.drive.MAX_VELOCITY-200, -150000, 0,0, 180, 0,3000);

//                robot.cycleV4(robot.a*robot.drive.TAG_CENTER_TO_CENTER,useArms,2,0);
                /*
                while(!driverGamepad.wasAPressed()){
                    driverGamepad.loop();
                    telemetry.addLine("Toggle Release");
                    telemetry.update();
                    if(driverGamepad.wasRightBumperPressed()){
                        robot.releaser.toggle();
                    }
                }
                robot.drive.setHeading(180);
                robot.drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Test new "Outside" path
                if(teamUtil.alliance == RED){
                    //robot.drive.moveCm(2000,56+robot.a,90,180,650);

                    robot.drive.strafeToEncoderWithDecel(90,180,2000,7300+robot.a,650, robot.drive.MAX_STRAFE_DECELERATION,2000);
                }else{
                    //robot.drive.moveCm(2000,59+robot.b,270,180,650);

                    robot.drive.strafeToEncoderWithDecel(270,180,2000,-7600+robot.a,650, robot.drive.MAX_STRAFE_DECELERATION,2000);
                }
                robot.drive.moveCm(robot.drive.MAX_VELOCITY,teamUtil.alliance == RED? 33:36,180,180,650);
                robot.releaser.release();
                if (useArms) {
                    robot.intake.startIntake();
                }
                robot.drive.moveCm(650,4,180,180,650);
                robot.drive.strafeToEncoderWithDecel(teamUtil.alliance == RED? 90:270,180,650, (teamUtil.alliance == RED? 1: -1)*12250,450, robot.drive.MAX_STRAFE_DECELERATION,1500 );
                robot.drive.moveCm(robot.drive.MAX_VELOCITY,13,180,180,0);

                 */

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


/*
                // Test April Tag Localization

                while(!driverGamepad.wasAPressed()){
                    driverGamepad.loop();
                    telemetry.addLine("Toggle CV");
                    telemetry.update();
                    if(driverGamepad.wasRightBumperPressed()){
                        robot.releaser.toggle();
                    }
                }
                int path = (int)robot.a;
                long startTime = System.currentTimeMillis();
                robot.drive.setHeading(180);
                robot.drive.strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.drive.forwardEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //robot.drive.strafeToAprilTagOffsetV3(400,5+robot.b,270,180,0,20,3000);
                int backupTarget;
                if(teamUtil.alliance==RED){
                    backupTarget = path==1? -188500 : path==2 ? -177500: -166500; // add 15 //path 2 = -177500 path 1 =-166500
                }else{
                    backupTarget = path==3? -188500 : path==2 ? -177500: -166500; // add 15 //path 2 = -177500 path 1 =-166500
                }

                robot.pushPurplePlaceYellowPixelWingV5(path,useArms);

                //robot.driveToBackDropV3(path, useArms,0* (teamUtil.alliance==RED ? 1 : -1),(int)(backupTarget+robot.b),true);
                if (useArms) {robot.output.dropAndGoToLoad();}
                elapsedTime = System.currentTimeMillis()-startTime;
                teamUtil.log("Elapsed Time Path "+ path+" : " + ((float)(System.currentTimeMillis()-startTime)/(float)1000));
//
//                /* from cycle stack */
                //int distance = path==1? 260 : path==2 ? 243: 225;
                /*
                int distance = path==1? 275 : path==2 ? 260: 240; // add 15

                robot.driveToBackDropV2(path, useArms,0* (teamUtil.alliance==RED ? 1 : -1),distance+robot.b,3,robot.output.GrabberRotatorHorizontal2, robot.output.StraferLoad);
                if (useArms) {robot.output.dropAndGoToLoad();}
                elapsedTime = System.currentTimeMillis()-startTime;
                teamUtil.log("Elapsed Time Path "+ path+" : " + ((float)(System.currentTimeMillis()-startTime)/(float)1000));

                 */

//
//                /* from wing stack */
//                robot.drive.moveCm(robot.drive.MAX_VELOCITY,2,0,180,750);
//                robot.intake.ready();
//                robot.drive.moveCm(robot.drive.MAX_VELOCITY,15,0,180,750);
//                int distance = path==1? 250 : path==2 ? 235: 215;
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
//                robot.driveToBackDropV2(path, useArms,4000* (teamUtil.alliance==RED ? 1 : -1),distance+robot.b,3,rotation, strafe); //was 17500//TODO adjust strafe for off wall
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
