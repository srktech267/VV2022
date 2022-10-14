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


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class VVRobotEX
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx  leftMotorFront  = null;
    public DcMotorEx  rightMotorFront = null;
    public DcMotorEx  leftMotorBack   = null;
    public DcMotorEx  rightMotorBack  = null;

    // to pick up rings
    public DcMotorEx  ringPickupMotor = null;
    public DcMotorEx  conveyorMotor   = null;

    // to shoot rings
    public DcMotorEx  shootingMotor   = null;

    // to pick up wobble goal
    public DcMotorEx  armMotor        = null;

    public Servo clawServo       = null;



    public DistanceSensor sensorRange = null;

    public VVRobotEX(HardwareMap hardwareMap) {

                 /* local OpMode members. */
        HardwareMap hwMap = hardwareMap;

        /* Constructor */

        /* Initialize standard Hardware interfaces */
        hwMap = hwMap;

        // Define and Initialize Motors
        //leftMotorFront = hwMap.get(DcMotorEx.class, "LMF");
        //rightMotorFront = hwMap.get(DcMotorEx.class, "RMF");
        //leftMotorBack = hwMap.get(DcMotorEx.class, "LMB");
        //rightMotorBack = hwMap.get(DcMotorEx.class, "RMB");
         /*armMotor = hwMap.get(DcMotorEx.class, "AM");
        ringPickupMotor = hwMap.get(DcMotorEx.class, "RPM");
        conveyorMotor = hwMap.get(DcMotorEx.class, "CM");
        shootingMotor = hwMap.get(DcMotorEx.class, "SM"); */


        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(DistanceSensor.class, "2MD");


        leftMotorFront.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotorEx.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotorBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorEx.Direction.REVERSE);
       // armMotor.setDirection(DcMotorEx.Direction.FORWARD);
       // ringPickupMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Set all motors to zero power
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

       /*
        armMotor.setPower(0);
        ringPickupMotor.setPower(0);
        conveyorMotor.setPower(0);
        shootingMotor.setPower(0);

        */

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotorFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        /*
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ringPickupMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        conveyorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
  */

        // Define and initialize ALL installed servos.
        // clawServo = hwMap.get(Servo.class, "CS");
    }



    public DistanceSensor getSensorRange() {
        return sensorRange;
    }

    public void setSensorRange(DistanceSensor sensorRange) {
        this.sensorRange = sensorRange;
    }
}




