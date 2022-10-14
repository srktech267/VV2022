/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class VVRobotOps extends LinearOpMode {

    //Override
    public void driveFront(VVRobot robot, double power) {


        //rightMotor = hardwareMap.dcMotor.get("RMF");
        try {

            robot.front_left_wheel.setPower(power);
            robot.front_right_wheel.setPower(power);
            robot.back_left_wheel.setPower(power);
            robot.back_right_wheel.setPower(power);


        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void driveFront(VVRobot robot, int driveTime, double maxPower, ElapsedTime runtime) {


        robot.setWheelsMoveForward();
        try {

            int drivetime = driveTime;
            double speed = 0;




            while (true)
            {
                if (speed <maxPower) {
                    speed = speed + .005;
                }
               setRobotSpeed(robot, speed);
                Log.i("vikingTech", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= drivetime /2)
                {
                    break;
                }
            }



            while (true)
            {
                if (speed > .1 ) {
                    speed = speed - .002;
                }


                setRobotSpeed(robot, speed);

                Log.i("vikingTech", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= drivetime )
                {
                    break;
                }
            }


            setRobotSpeed(robot, 0);
        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void driveBack(VVRobot robot, int driveTime, double maxPower, ElapsedTime runtime) {


        //rightMotor = hardwareMap.dcMotor.get("RMF");
        try {
            robot.setWheelsMoveBackward();
            int drivetime = driveTime;
            double speed = 0;

            while (true)
            {
                if (speed <maxPower) {
                    speed = speed + .005;
                }
                setRobotSpeed(robot, speed);
                Log.i("vikingTech", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= drivetime /2)
                {
                    break;
                }
            }



            while (true)
            {
                if (speed > .1 ) {
                    speed = speed - .002;
                }


                setRobotSpeed(robot, speed);

                Log.i("vikingTech", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= drivetime )
                {
                    break;
                }
            }


            setRobotSpeed(robot, 0);
        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void slideRight(VVRobot robot, double power, int time)
    {
        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

    

        robot.front_left_wheel.setPower(power);
        robot.front_right_wheel.setPower(power);
        robot.back_left_wheel.setPower(-power);
        robot.back_right_wheel.setPower(-power);
        

    }
    
    public void slideRightdemo(VVRobot robot, double power, int time)
    {
     //  robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
    //    robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
       // robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

    

        robot.front_left_wheel.setPower(power);
        robot.front_right_wheel.setPower(0);
        
        robot.back_left_wheel.setPower(0);
        robot.back_right_wheel.setPower(0);
        

    }

    public void slideLeft(VVRobot robot, double power, int time)
    {
        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.front_left_wheel.setPower(power);
        robot.front_right_wheel.setPower(power);
        robot.back_left_wheel.setPower(power);
        robot.back_right_wheel.setPower(power);

    }
    public void setRobotSpeed(VVRobot robot, double speed)
    {

        robot.front_left_wheel.setPower(speed);
        robot.front_right_wheel.setPower(speed);
        robot.back_left_wheel.setPower(speed);
        robot.back_right_wheel.setPower(speed);
    }
    public void driveFront(VVRobot robot, double power, int time ) {


        //rightMotor = hardwareMap.dcMotor.get("RMF");
        try {


            robot.front_left_wheel.setPower(power);
            robot.front_right_wheel.setPower(power);
            robot.back_left_wheel.setPower(power);
            robot.back_right_wheel.setPower(power);
            sleep(time);
            Stop(robot);


        } catch (Exception e) {
            e.printStackTrace();

        }
    }
    public void driveBack(VVRobot robot, double power)  {

        try {

        //rightMotor = hardwareMap.dcMotor.get("RMF");
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.front_left_wheel.setPower(power);
        robot.front_right_wheel.setPower(power);
        robot.back_left_wheel.setPower(power);
        robot.back_right_wheel.setPower(power);
        }catch (Exception e)
        {
          e.printStackTrace();
        }


    }

    public void driveBack(VVRobot robot, double power, int time)  {

        try {

            //rightMotor = hardwareMap.dcMotor.get("RMF");
            robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

            robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

            robot.front_left_wheel.setPower(power);
            robot.front_right_wheel.setPower(power);
            robot.back_left_wheel.setPower(power);
            robot.back_right_wheel.setPower(power);
            sleep(time);
            Stop(robot);

        }catch (Exception e)
        {
            e.printStackTrace();
        }


    }
    public void Stop(VVRobot robot) {

        //Log.i("vikingTech","robot stopping");
        robot.front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.front_left_wheel.setPower(0);
        robot.front_right_wheel.setPower(0);
        robot.back_left_wheel.setPower(0);
        robot.back_right_wheel.setPower(0);
    }


    public void turn(VVRobot robot, double power , int deg)  {

        //rightMotor = hardwareMap.dcMotor.get("RMF");
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.front_left_wheel.setPower(power);
        robot.front_right_wheel.setPower(-power);
        robot.back_left_wheel.setPower(power);
        robot.back_right_wheel.setPower(-power);

        //run for .875 seconds
        sleep(20*deg);

    }
    public void setRobotMotorPower(VVRobot robot, double rightPower , double leftPower)  {

        //rightMotor = hardwareMap.dcMotor.get("RMF");
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.front_left_wheel.setPower(leftPower);
        robot.back_left_wheel.setPower(leftPower);

        robot.back_right_wheel.setPower(rightPower);
        robot.front_right_wheel.setPower(rightPower);

    }
    public void setRobotMotorPower(VVRobot robot, double front_right_wheel , double front_left_wheel , double back_right_wheel, double back_left_wheel)  {


        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);


        robot.front_right_wheel.setPower(front_right_wheel);
        robot.front_left_wheel.setPower(front_left_wheel);

        robot.back_right_wheel.setPower(back_right_wheel);
        robot.back_left_wheel.setPower(back_left_wheel);




    }
    public void turn(VVRobot robot, double rightPower , double leftPower)  {

        //rightMotor = hardwareMap.dcMotor.get("RMF");
        robot.front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.front_left_wheel.setPower(leftPower);
        robot.back_left_wheel.setPower(leftPower);

        robot.back_right_wheel.setPower(rightPower);
        robot.front_right_wheel.setPower(rightPower);

    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
