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


    private ElapsedTime     runtime = new ElapsedTime();
    
    
    public void driveFront(VVRobot robot, double power,int driveTime) {
       
        try {


            runtime.reset();
            

            while (true)
            {
                robot.setWheelsMoveForward();
                robot.setWheelsSpeed(power,power,power,power);

                Log.i("VVRobot", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= driveTime )
                {
                    robot.stop();
                    break;
                }
            }
                       

        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void driveBack(VVRobot robot, double power,int driveTime) {

        try {


            runtime.reset();
            

            while (true)
            {
                robot.setWheelsMoveBackward();
                robot.setWheelsSpeed(power,power,power,power);

                Log.i("VVRobot", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= driveTime )
                {
                    robot.stop();
                    break;
                }
            }


        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void driveFrontWithSpeedControl(VVRobot robot, int driveTime, double maxPower) {


        robot.setWheelsMoveForward();
        try {

             
            double speed = 0;

            runtime.reset();
            

            while (true)
            {
                if (speed <maxPower) {
                    speed = speed + .005;
                }
               setRobotSpeed(robot, speed);
                Log.i("VVRobot", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= driveTime /2)
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

                Log.i("VVRobot", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= driveTime )
                {
                    break;
                }
            }


            robot.stop();
        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void driveBackWithSpeedControl(VVRobot robot, int driveTime, double maxPower ) {


        //rightMotor = hardwareMap.dcMotor.get("RMF");
        try {
            runtime.reset();
            
            robot.setWheelsMoveBackward();
           
            double speed = 0;

            while (true)
            {
                if (speed <maxPower) {
                    speed = speed + .005;
                }
                setRobotSpeed(robot, speed);
                Log.i("VVRobot", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= driveTime /2)
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

                Log.i("VVRobot", "runtime" + runtime.milliseconds() );
                if ( runtime.milliseconds() >= driveTime )
                {
                    break;
                }
            }


            robot.stop();
        } catch (Exception e) {
            e.printStackTrace();

        }
    }

    public void slideRight(VVRobot robot, double power, int driveTime)
    {
        robot.setWheelsMoveForward();

       
       runtime.reset();


             robot.setWheelsSpeed(power,-power,(-power*.9),(power*.9));

             sleep(driveTime);


        
        

    }


    public void slideLeft(VVRobot robot, double power, int driveTime)
    {
        robot.setWheelsMoveForward();




            robot.setWheelsSpeed(-power,power,(power*.9),(-power*.9));

            sleep(driveTime);




    }



    public void setRobotSpeed(VVRobot robot, double speed)
    {

        robot.front_left_wheel.setPower(speed);
        robot.front_right_wheel.setPower(speed);
        robot.back_left_wheel.setPower(speed);
        robot.back_right_wheel.setPower(speed);
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
 
     

    public void turn(VVRobot robot,int direction ,int degrees)
    {

        double power = .3;
        robot.setWheelsMoveForward();




       if (direction <0)
       {
           robot.setWheelsSpeed(-power,power,-power,power);
           sleep (degrees);
       }else if(direction>0)
       {
           robot.setWheelsSpeed(power,-power,power,-power);
           sleep (degrees);
       }




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
