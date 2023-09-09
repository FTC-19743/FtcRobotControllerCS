package org.firstinspires.ftc.teamcode.assemblies.OLD;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public static boolean armsCalibrated = false;
    //public TwoWheelDrive drive;
    public OutakeArm outakeArm;
    public OutakeSlide outakeSlide;
    //public CarouselSpinner spinner;
    //public TSEDetector detector;
    //public OpenCVDetector detector;


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Robot() {
        teamUtil.log("Constructing Robot");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        //drive = new TwoWheelDrive();
        outakeArm = new OutakeArm();
        outakeSlide = new OutakeSlide();
        //spinner = new CarouselSpinner();
        //detector = new TSEDetector();
        //detector = new OpenCVDetector();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void init(boolean auto) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //These are the parameters that the imu uses in the code to name and keep track of the data
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        teamUtil.log("Initializing Robot");
        //drive.initialize();
        outakeArm.init();
        if(auto == true) {
            outakeArm.resetArm();
            armsCalibrated = true;
        }
        //outakeSlide.init();

        //outakeSlide.resetSlider();
        //spinner.init();

    }

    public void doAuto(int path) {
        //code for blue alliance
        if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
            if (path != 0) {
                //double startingIMU = drive.getIMUHeading();
                /*
                outakeArm.spinnerOutput();
                teamUtil.pause(100);
                outakeArm.spinnerStop();

                 */

                //teamUtil.pause(500);
                if (path == 1) {
                    outakeArm.runToFirstLevelAuto();
                } else if (path == 2) {
                    outakeArm.runToSecondLevelAuto();
                } else {
                    outakeArm.runToThirdLevelAuto();
                }
                //drive.moveInches(.3, 39);
                //drive.spinLeftWithIMUV2(90, .25);

                if (path == 1) {
                    //drive.moveInches(.25, 6.5);
                } else if (path == 2) {
                    //drive.moveInches(.25, 6.25);
                } else {
                    //drive.moveInches(.25, 6.5);
                }

                outakeArm.spinnerOutput();
                teamUtil.pause(1000);
                outakeArm.spinnerStop();

                //outakeArm.runToTop();
                if (path == 1) {
                    //drive.moveBackInches(.4, 33);
                    outakeArm.runToGround();
                    //drive.moveBackInches(.4, 35);
                } else if (path == 2) {
                    //drive.moveBackInches(.4, 32);
                    outakeArm.runToGround();
                    //drive.moveBackInches(.25,3);
                } else {
                    //drive.moveBackInches(.4, 33.5);
                    outakeArm.runToGround();
                    //drive.moveBackInches(.25,2);
                }

                teamUtil.pause(250);
                //drive.motorsOn(-.25);
                teamUtil.pause(2000);
//                drive.motorsOff();
//                drive.moveInches(.2, 5);
//                drive.spinRightWithIMUV2(90, .2);
//                drive.moveBackInches(.35, 29);

                teamUtil.pause(250);

//                drive.motorsOn(-.03);
//                spinner.on(.40);
                teamUtil.pause(7500);
//                spinner.off();
//                drive.motorsOff();
//                double currentIMU = drive.getIMUHeading();
//                double degreesNeeded = currentIMU - startingIMU;
//                if (degreesNeeded < 0) {
//                    degreesNeeded = degreesNeeded * -1;
//                    drive.spinLeftWithIMUV2(degreesNeeded, .25);
//                } else {
//                    drive.spinRightWithIMUV2(degreesNeeded, .25);
//                }
//
//                drive.moveInches(.35, 18);

            }
        }

        //code for red alliance
        else {

            if (path != 0) {
                //double startingIMU = drive.getIMUHeading();

                // outakeArm.spinnerOutput();
                // teamUtil.pause(100);
                // outakeArm.spinnerStop();


                //teamUtil.pause(500);
                if (path == 1) {
                    outakeArm.runToFirstLevelAuto();
                } else if (path == 2) {
                    outakeArm.runToSecondLevelAuto();
                } else {
                    outakeArm.runToThirdLevelAuto();
                }
//                drive.moveInches(.3, 39);
//                drive.spinRightWithIMUV2(90, .25);

                if (path == 1) {
                    //drive.moveInches(.25, 6.25  );
                } else if (path == 2) {
                    //drive.moveInches(.25, 6.5);
                } else {
                    //drive.moveInches(.25, 6.5);
                }

                outakeArm.spinnerOutput();
                teamUtil.pause(1000);
                outakeArm.spinnerStop();

                //outakeArm.runToTop();
                if (path == 1) {
                    //drive.moveBackInches(.4, 33);
                    outakeArm.runToGround();
                    //drive.moveBackInches(.25, 2);
                    //drive.moveBackInches(.4, 35);
                } else if (path == 2) {
                    //drive.moveBackInches(.4, 33);
                    outakeArm.runToGround();
                    //drive.moveBackInches(.25,3);
                } else {
                    //drive.moveBackInches(.4, 33.5);
                    outakeArm.runToGround();
                    //drive.moveBackInches(.25,2);
                }

                teamUtil.pause(250);
//                drive.motorsOn(-.25);
                teamUtil.pause(1000);
//                drive.motorsOff();
//                drive.moveInches(.2, 7);
//                drive.spinLeftWithIMUV2(90, .2);
//                drive.moveBackInches(.35, 34);
                //Turn towards carousel
//                drive.spinRightWithIMUV2(60,.2);
//                drive.motorsOn(-.2);
                teamUtil.pause(500);
//                drive.motorsOn(-.02);
//                spinner.on(.40);
                teamUtil.pause(5500);
//                spinner.off();
//                drive.motorsOff();
//                double currentIMU = drive.getIMUHeading();
//                double degreesNeeded = currentIMU + startingIMU;
//                if (degreesNeeded < 0) {
//                    degreesNeeded = degreesNeeded * -1;
//                    drive.spinLeftWithIMUV2(degreesNeeded, .25);
//                } else {
//                    drive.spinRightWithIMUV2(degreesNeeded, .25);
//                }
//
//                drive.moveInches(.35, 18);
                //DRAFT DUCK INTAKE CODE
                /*
                drive.moveInches(.35,10);
                drive.spinRightWithIMUV2(135,.25);

                 */




        /*
        outakeArm.spinnerOutput();
                    teamUtil.pause(100);
                    outakeArm.spinnerStop();
                    drive.moveInches(.3, 36);
                    //teamUtil.pause(500);
                    if (path == 1) {
                       // outakeArm.runArmToPosition(outakeArm.Level1);
                    } else if (path == 2) {
                        outakeArm.runArmToPosition(outakeArm.Level2);
                    } else {
                        outakeArm.runArmToPosition(outakeArm.Level3);
                    }

                    drive.spinRightWithIMUV2(90, .1);

                    if (path == 1) {
                        drive.moveBackInches(.5,2);
                        outakeArm.runArmToPosition(outakeArm.Level1);
                        teamUtil.pause(2000);
                        drive.moveInches(.25, 2);
                    } else if (path == 2) {
                        drive.moveInches(.25, 5);
                    } else {
                        drive.moveInches(.25, 6.5);
                    }
                    outakeArm.spinnerOutput();
                    teamUtil.pause(1000);
                    outakeArm.spinnerStop();


                    outakeArm.runToTop();
                    if (path == 1) {
                        drive.moveBackInches(.4, 25.5);
                    } else if (path == 2) {
                        drive.moveBackInches(.4, 29.5);
                    } else {
                        drive.moveBackInches(.4, 34.5);
                    }

                    teamUtil.pause(1000);
                    drive.moveInches(.25, 5);
                    drive.spinRightWithIMUV2(90, .1);
                    drive.moveInches(.35, 28);

                    teamUtil.pause(1000);
                    drive.spinRightWithIMUV2(45, .1);
                    drive.moveInches(.35, 11);
                    drive.motorsOn(.05);
                    spinner.on(.30);
                    teamUtil.pause(4500);
                    spinner.off();
                    drive.motorsOff();
                    drive.spinLeftWithIMUV2(45, .25);
                    drive.moveBackInches(.35, 17);

                 */
            }
        }
    }
}



