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

package org.firstinspires.ftc.teamcode.core;


import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware. Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


// based Mech setup and gear positions , FL and BL should have opposite polarity / speed/ direction
//FL,FR
//BL,BR
//1,2
//3,4
//-,+
//-,+



public class VVRobot
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor getFront_left_wheel() {
        return front_left_wheel;
    }

    public void setFront_left_wheel(DcMotor front_left_wheel) {
        this.front_left_wheel = front_left_wheel;
    }

    public DcMotor getFront_right_wheel() {
        return front_right_wheel;
    }

    public void setFront_right_wheel(DcMotor front_right_wheel) {
        this.front_right_wheel = front_right_wheel;
    }

    public DcMotor getBack_left_wheel() {
        return back_left_wheel;
    }

    public void setBack_left_wheel(DcMotor back_left_wheel) {
        this.back_left_wheel = back_left_wheel;
    }

    public DcMotor getBack_right_wheel() {
        return back_right_wheel;
    }

    public void setBack_right_wheel(DcMotor back_right_wheel) {
        this.back_right_wheel = back_right_wheel;
    }

    public DcMotor  front_left_wheel = null;
    public DcMotor  front_right_wheel = null;
    public DcMotor  back_left_wheel   = null;
    public DcMotor  back_right_wheel  = null;


    public Servo getConePickupServo() {
        return conePickupServo;
    }

    // to pick up rings
    public  Servo conePickupServo= null;
    public DcMotor  conveyorMotor   = null;

    // to shoot rings
    public DcMotor  shootingMotor   = null;

    // to pick up wobble goal
    public DcMotor  armMotor        = null;



    public BNO055IMU imu;

    public DistanceSensor sensorRange = null;
    public HardwareMap hardwareMap = null;

    public VVRobot(HardwareMap hardwareMap, String mode, boolean useIMU)
    {
       this.hardwareMap = hardwareMap;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        VikingTechRobotInit(hardwareMap);


    }
    public VVRobot(HardwareMap hardwareMap)
    {

        this.hardwareMap = hardwareMap;
        VikingTechRobotInit(hardwareMap);
    }
    private void VikingTechRobotInit(HardwareMap hardwareMap)
    {

               /* Constructor */



        // Define and Initialize Motors
        front_left_wheel = hardwareMap.get(DcMotor.class, VVHardwareMapping.front_left_wheel);
        back_left_wheel = hardwareMap.get(DcMotor.class, VVHardwareMapping.back_left_wheel);
        front_right_wheel = hardwareMap.get(DcMotor.class, VVHardwareMapping.front_right_wheel);
        back_right_wheel = hardwareMap.get(DcMotor.class, VVHardwareMapping.back_right_wheel);

        if (VVHardwareMapping.conePickupServo != null)
        {
            conePickupServo =  hardwareMap.get(Servo.class,VVHardwareMapping.conePickupServo);
            conePickupServo.getController().pwmEnable();

        }


        if (VVHardwareMapping.armMotor != null)
        {
            armMotor = hardwareMap.get(DcMotor.class, VVHardwareMapping.armMotor);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setDirection(DcMotor.Direction.FORWARD);
        }



        front_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        front_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    public void setWheelsDirections(DcMotorSimple.Direction FLW, DcMotorSimple.Direction FRW,DcMotorSimple.Direction BLW
            ,DcMotorSimple.Direction BRW)
    {
        front_left_wheel.setDirection(FLW);
        front_right_wheel.setDirection(FRW);
        //front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //given the drive side location , we have to reverse the direction


        // back_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //given the drive side location , we have to reverse the direction
        back_left_wheel.setDirection(BLW);
        back_right_wheel.setDirection(BRW);

    }

    public void setWheelsSpeed(double FLWP, double FRWP,double BLWP
            ,double BRWP)
    {

        //front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //given the drive side location , we have to reverse the direction
        front_left_wheel.setPower(FLWP);
        front_right_wheel.setPower(FRWP);

        // back_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //given the drive side location , we have to reverse the direction
        back_left_wheel.setPower(BLWP);
        back_right_wheel.setPower(BRWP);

    }

    public void stop()
    {
        front_left_wheel.setPower(0);
        front_right_wheel.setPower(0);
        back_left_wheel.setPower(0);
        back_right_wheel.setPower(0);

    }

    public void setWheelsMoveBackward()
    {

        //given the drive side location , we have to reverse the direction
        front_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);

        //given the drive side location , we have to reverse the direction
        back_right_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_wheel.setDirection(DcMotorSimple.Direction.FORWARD);



    }

    public void
    setWheelsMoveForward()
    {

        // based Mech setup and gear positions , FL and BL should have opposite polarity / speed/ direction
//FL,FR
//BL,BR
//1,2
//3,4
//-,+
//-,+
        //given the drive side location , we have to reverse the direction
        front_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);


        //given the drive side location , we have to reverse the direction
        back_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public void setWheelsAllForward()
    {

        front_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //given the drive side location , we have to reverse the direction
        front_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //given the drive side location , we have to reverse the direction
        back_right_wheel.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_wheel.setDirection(DcMotorSimple.Direction.REVERSE);



    }


    public void checkWheelAlignment(DcMotor motor, DcMotorSimple.Direction dir, double speed)
    {

        motor.setDirection(dir);
        motor.setPower(speed);


    }






}




