package org.firstinspires.ftc.teamcode.testCode;


import static java.lang.Math.toDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Drive;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "SplineTest", group = "LinearOpMode")
public class TestSpline extends LinearOpMode {
    TeamGamepad gamepad;
    Robot robot;
    Drive drive;
    double MIN_SPLINE_DISTANCE = 1;

    public void runSpline(double maxVelocity, double[] firstPosition, double[] secondPosition, double[] thirdPosition, double[] fourthPosition, boolean moveForwards, double endVelocity, long timeoutTime){
        // positions are {forwards encoder tics, side encoder tics}
        double decelVelocityChange = maxVelocity - endVelocity;
        double decelCm = Math.abs(decelVelocityChange / robot.drive.MAX_DECELERATION)/robot.drive.COUNTS_PER_CENTIMETER; // covert tics to cm because inconsistent tics
        double decelPerCm = decelVelocityChange/decelCm; // decel in cm because of inconsistent tics
        double startHeading = robot.drive.getHeading(); // heading when slope = 0
        if(endVelocity < robot.drive.MIN_END_VELOCITY){
            endVelocity = robot.drive.MIN_END_VELOCITY;
        }
        if(!moveForwards){ // account for direction
            startHeading = robot.drive.adjustAngle(startHeading-180);
        }
        if(firstPosition[0] > secondPosition[0]){
            teamUtil.log("Spine points not in order");
            return;
        }
        if(secondPosition[0] > thirdPosition[0]){
            teamUtil.log("Spine points not in order");
            return;
        }
        if(thirdPosition[0] > fourthPosition[0]){
            teamUtil.log("Spine points not in order");
            return;
        }
        double a, b, c, d;
        double xOffset = 0;
        double yOffset = 0;
        double[][] positions = {firstPosition, secondPosition, thirdPosition, fourthPosition};
        for(int i = 0; i < 4; i++) { // make x and y offset to make solving easier
            if(i == 0){
                xOffset = positions[0][0];
                yOffset = positions[0][1];
            }else{
                if(positions[i][0] < xOffset){
                    xOffset = positions[i][0];
                }
                if(positions[i][1] < yOffset){
                    yOffset = positions[i][1];
                }
            }
        }
        teamUtil.log("yOffset:"+yOffset+" xOffset:"+xOffset);
        d = firstPosition[1]-yOffset;
        // offests simplify the first row of the matrix by immediately finding d
        //rows are in a^3 b^2 c d y value
        double[] secondRow = {Math.pow((secondPosition[0]-xOffset), 3), Math.pow((secondPosition[0]-xOffset), 2), secondPosition[0]-xOffset, d, secondPosition[1]-yOffset};
        double[] thirdRow = {Math.pow((thirdPosition[0]-xOffset), 3), Math.pow((thirdPosition[0]-xOffset), 2), thirdPosition[0]-xOffset, d, thirdPosition[1]-yOffset};
        double[] fourthRow = {Math.pow((fourthPosition[0]-xOffset), 3), Math.pow((fourthPosition[0]-xOffset), 2), fourthPosition[0]-xOffset, d, fourthPosition[1]-yOffset};

        // clear first pivot column
        double factor = thirdRow[0]/secondPosition[0];
        thirdRow = new double[]{thirdRow[0]-secondRow[0]*factor, thirdRow[1]-secondRow[1]*factor, thirdRow[2]-secondRow[2]*factor, thirdRow[3]-secondRow[3]*factor, thirdRow[4]-secondRow[4]*factor};

        factor = fourthRow[0]/secondPosition[0];
        fourthRow = new double[]{fourthRow[0]-secondRow[0]*factor, fourthRow[1]-secondRow[1]*factor, fourthRow[2]-secondRow[2]*factor, fourthRow[3]-secondRow[3]*factor, fourthRow[4]-secondRow[4]*factor};

        //lead with 1
        factor = 1/secondRow[0];
        secondRow = new double[]{secondRow[0]*factor, secondRow[1]*factor, secondRow[2]*factor, secondRow[3]*factor, secondRow[4]*factor};

        // clear second pivot column
        factor = fourthRow[1]/thirdRow[1];
        fourthRow = new double[]{fourthRow[0]-thirdRow[0]*factor, fourthRow[1]-thirdRow[1]*factor, fourthRow[2]-thirdRow[2]*factor, fourthRow[3]-thirdRow[3]*factor, fourthRow[4]-thirdRow[4]*factor};

        // lead with 1
        factor = 1/thirdRow[1];
        thirdRow = new double[]{thirdRow[0]*factor, thirdRow[1]*factor, thirdRow[2]*factor, thirdRow[3]*factor, thirdRow[4]*factor};

        // lead with 1
        factor = 1/fourthRow[2];
        fourthRow = new double[]{fourthRow[0]*factor, fourthRow[1]*factor, fourthRow[2]*factor, fourthRow[3]*factor, fourthRow[4]*factor};

        // get rid of stuff above pivots
        factor = thirdRow[2]/fourthPosition[2];
        thirdRow = new double[]{thirdRow[0]-fourthRow[0]*factor, thirdRow[1]-fourthRow[1]*factor, thirdRow[2]-fourthRow[2]*factor, thirdRow[3]-fourthRow[3]*factor, thirdRow[4]-fourthRow[4]*factor};

        factor = secondRow[2]/fourthPosition[2];
        secondRow = new double[]{secondRow[0]-fourthRow[0]*factor, secondRow[1]-fourthRow[1]*factor, secondRow[2]-fourthRow[2]*factor, secondRow[3]-fourthRow[3]*factor, secondRow[4]-fourthRow[4]*factor};

        factor = secondRow[1]/thirdPosition[1];
        secondRow = new double[]{secondRow[0]-thirdRow[0]*factor, secondRow[1]-thirdRow[1]*factor, secondRow[2]-thirdRow[2]*factor, secondRow[3]-thirdRow[3]*factor, secondRow[4]-thirdRow[4]*factor};

        // isolate a/b/c = a number
        a = secondRow[4] - secondRow[3];
        b = thirdRow[4] - secondRow[3];
        c = thirdRow[4] - secondRow[3];
        teamUtil.log("a: "+a+" b: "+b+" c: "+c+" d: "+d);
        double totalDistance = Math.sqrt(Math.pow((robot.drive.forwardEncoder.getCurrentPosition()-fourthPosition[0])/robot.drive.TICS_PER_CM_STRAIGHT_ENCODER, 2)+Math.pow((robot.drive.strafeEncoder.getCurrentPosition()-fourthPosition[1])/robot.drive.TICS_PER_CM_STRAFE_ENCODER, 2)); // in cms;
        double distance = Math.sqrt(Math.pow((robot.drive.forwardEncoder.getCurrentPosition()-fourthPosition[0])/robot.drive.TICS_PER_CM_STRAIGHT_ENCODER, 2)+Math.pow((robot.drive.strafeEncoder.getCurrentPosition()-fourthPosition[1])/robot.drive.TICS_PER_CM_STRAFE_ENCODER, 2)); // in cms
        while(teamUtil.keepGoing(timeoutTime) && distance > decelCm){
            distance = Math.sqrt(Math.pow((robot.drive.forwardEncoder.getCurrentPosition()-fourthPosition[0])/robot.drive.TICS_PER_CM_STRAIGHT_ENCODER, 2)+Math.pow((robot.drive.strafeEncoder.getCurrentPosition()-fourthPosition[1])/robot.drive.TICS_PER_CM_STRAFE_ENCODER, 2));
            // strafe is dependant on forwards
            double forwardsPosition = robot.drive.forwardEncoder.getCurrentPosition()-xOffset;
            // find slope, coords, and angle for unit circle
            double slope = 3*a*Math.pow(forwardsPosition, 2) + 2*b*forwardsPosition+c;
            double circleX = 1/(Math.sqrt(1+Math.pow(slope, 2)));
            double circleY = slope/(Math.sqrt(1+Math.pow(slope, 2)));
            double lineAngle = Math.toDegrees(Math.atan(circleY/circleX)); // angle from the x axis to the line
            if(slope > 0){ // adjust turn direction
                lineAngle *= -1;
            }
            robot.drive.driveMotorsHeadingsFR(robot.drive.adjustAngle(lineAngle+startHeading), robot.drive.adjustAngle(lineAngle+startHeading), maxVelocity);
        }
        while(teamUtil.keepGoing(timeoutTime) && distance >= MIN_SPLINE_DISTANCE){
            distance = Math.sqrt(Math.pow((robot.drive.forwardEncoder.getCurrentPosition()-fourthPosition[0])/robot.drive.TICS_PER_CM_STRAIGHT_ENCODER, 2)+Math.pow((robot.drive.strafeEncoder.getCurrentPosition()-fourthPosition[1])/robot.drive.TICS_PER_CM_STRAFE_ENCODER, 2));
            // strafe is dependant on forwards
            double forwardsPosition = robot.drive.forwardEncoder.getCurrentPosition()-xOffset;
            // find slope, coords for angle in unit circle
            double slope = 3*a*Math.pow(forwardsPosition, 2) + 2*b*forwardsPosition+c;
            double circleX = 1/(Math.sqrt(1+Math.pow(slope, 2)));
            double circleY = slope/(Math.sqrt(1+Math.pow(slope, 2)));
            double lineAngle = Math.toDegrees(Math.atan(circleY/circleX)); // angle from the x axis to the line
            if(slope > 0){ // adjust turn direction
                lineAngle *= -1;
            }
            robot.drive.driveMotorsHeadingsFR(robot.drive.adjustAngle(lineAngle+startHeading), robot.drive.adjustAngle(lineAngle+startHeading), (totalDistance-distance)*decelPerCm+robot.drive.MIN_END_VELOCITY);
        }
        robot.drive.stopMotors();
        robot.drive.lastVelocity = endVelocity;
    }

    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        robot.initialize();

        waitForStart();
        while (opModeIsActive()) {
            gamepad.loop();
            if(gamepad.wasAPressed()){
                runSpline(robot.drive.MAX_VELOCITY, new double[] {0,0},new double[] {0,0}, new double[] {0,0}, new double[] {0,0}, true, 0, 10000);
            }
        }
    }
}
