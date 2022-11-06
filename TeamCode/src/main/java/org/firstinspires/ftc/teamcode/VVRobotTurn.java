package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class VVRobotTurn extends LinearOpMode

{
    VVRobot robot = null;
    VVRobotOps robotOps = null;
    Orientation lastAngles = new Orientation();

    BNO055IMU imu;

    double globalAngle, power = .30, correction;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public VVRobotTurn(VVRobot robot, VVRobotOps robotOps) {

        this.robot =  robot;
        this.imu = robot.imu;
        this.robotOps = robotOps;  
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.
        robotOps.turn(robot, rightPower,leftPower);


        // rotate until turn is completed.

        Log.i("vikingTech", "IN rotate method : degrees:"+ degrees);
        Log.i("vikingTech", "Is opModeIsActive in rotate method:"+ opModeIsActive());
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while ( getAndPrintAngle() == 0) {

            }

            while ( getAndPrintAngle() > degrees) {

            }
        } else    // left turn.
            while (  getAndPrintAngle()  < degrees) {

            }

        // turn the motors off.
       robot.stop();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private double getAndPrintAngle()
    {
        double angleReturnedbyGetAngle;
        angleReturnedbyGetAngle=getAngle();
        Log.i("vikingTech", "angleReturnedbyGetAngle :"+ angleReturnedbyGetAngle);
        return angleReturnedbyGetAngle;

    }

    public void turnLeft(double turnAngle,Object obj) {
    sleep(500);

    Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double speed=.5;
    double oldDegreesLeft=turnAngle;
    double scaledSpeed=speed;
    double targetHeading= angles.firstAngle+turnAngle;
    double oldAngle = angles.firstAngle;
    if(targetHeading<-180) {targetHeading+=360;}
    if(targetHeading>180){targetHeading-=360;}
    double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
    //  runtime.reset();

   LinearOpMode opMode = (LinearOpMode)obj;


        Log.i("vikingTech", "condition1 :" + (opMode.opModeIsActive() && degreesLeft > 1 && oldDegreesLeft - degreesLeft >= 0));

        while (opMode.opModeIsActive() && degreesLeft > 1 && oldDegreesLeft - degreesLeft >= 0) {
                Log.i("vikingTech", "condition 2:" + (opMode.opModeIsActive() && degreesLeft > 1 && oldDegreesLeft - degreesLeft >= 0));

                //check to see if we overshot target
                scaledSpeed = degreesLeft / (100 + degreesLeft) * speed;
                if (scaledSpeed > 1) {
                    scaledSpeed = .1;
                }
                robot.back_left_wheel.setPower(scaledSpeed * 1.3); //extra power to back wheels
                robot.back_right_wheel.setPower(-1 * scaledSpeed * 1.3); //due to extra weight
                robot.front_left_wheel.setPower(scaledSpeed);
                robot.front_right_wheel.setPower(-1 * scaledSpeed);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                oldDegreesLeft = degreesLeft;
                degreesLeft = ((int) (Math.signum(angles.firstAngle - targetHeading) + 1) / 2) * (360 - Math.abs(angles.firstAngle - targetHeading)) + (int) (Math.signum(targetHeading - angles.firstAngle) + 1) / 2 * Math.abs(angles.firstAngle - targetHeading);
                Log.i("vikingTech", "degrees Left :" + degreesLeft);

                if (Math.abs(angles.firstAngle - oldAngle) < 1) {
                    speed *= 1.1;
                }
                ;
                //bump up speed to wheels in case robot stalls before reaching target
                oldAngle = angles.firstAngle;
            Log.i("vikingTech", "condition 1:" + (opMode.opModeIsActive() ));
            Log.i("vikingTech", "condition 2:" + (degreesLeft  ));
            Log.i("vikingTech", "condition 3:" + (oldDegreesLeft ));
            Log.i("vikingTech", "condition full:" + (opMode.opModeIsActive() && degreesLeft > 1 && oldDegreesLeft - degreesLeft >= 0));

            }

    robot.stop(); //our helper method to set all wheel motors to zero
    sleep(250); //small pause at end of turn
}
}

