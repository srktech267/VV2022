package org.firstinspires.ftc.teamcode.teleop;


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



import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.VVRobot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="VVTeleOpPWP", group="Tele-op")

public class VVTeleOpPWP extends LinearOpMode {



    private ElapsedTime runtime = new ElapsedTime();

    // Initialize our local variables for use later in telemetry or other methods
    public double y;
    public double x;
    public double rx;
    public double leftFrontPower;
    public double leftRearPower;
    public double rightFrontPower;
    public double rightRearPower;
    public double armPower = -.6;
    public double robotSpeed;
    public boolean constrainMovement;

    public int baseEncoderCount =0;
    public int currentEncoderCount =0;
    public int maxEncoderCount = 2800;

    // Initialize our local variables with values
    // These "slow" variable is used to control the overall speed of the robot
    // TODO: Work with Drive Team to determine
    public double baseSpeed = 0.9;

    public int lowJunction  = -1200;
    public int midJunction  = -2000;
    public int highJunction = -2600;
    public int STARTING_POSITION = 0;
    VVRobot robot;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot = new VVRobot(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Initialized");

        Servo servo = robot.getConePickupServo();

        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        baseEncoderCount = robot.armMotor.getCurrentPosition();


        // Wait for the game to start (driver presses PLAY)
        servo.getController().pwmEnable();

        // Configure initial variables
        constrainMovement = false;
        //Wait for the driver to press PLAY on the driver station phone
        // make a new thread
        telemetry.addData("Status", "Fully Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // based Mech setup and gear positions , FL and BL should have opposite polarity / speed/ direction
        //FL,FR
        //BL,BR
        //1,2
        //3,4
        //-,+
        //-,+

        // Polling rate for logging gets set to zero before the while loop
        int i = 0;

        try {



            while (opModeIsActive())
            {


                if (gamepad2.left_bumper)
                {
                    // Set the servo to the new position and pause;
                    //open
                    Log.e("VVTeleOpPWP", "Before open : "+servo.getPosition());
                    servo.setDirection(Servo.Direction.REVERSE);
                    servo.setPosition(1);


                    Log.e("VVTeleOpPWP", "after open : "+servo.getPosition());


                }
                else if  (gamepad2.right_bumper)
                {
                    // Set the servo to the new position and pause;
                    //close

                    Log.e("VVTeleOpPWP", "Before close : "+servo.getPosition());
                    servo.setDirection(Servo.Direction.FORWARD);
                    servo.setPosition(1);

                    Log.e("VVTeleOpPWP", "After Close : "+servo.getPosition());

                }

                currentEncoderCount = robot.armMotor.getCurrentPosition();
                Log.e("VVTeleOpPWP" , "Clicks =" +  currentEncoderCount );
                telemetry.addData("encode count==>" ,  currentEncoderCount );
                telemetry.update();
// Everything gamepad 1:
                // User controls for the robot speed overall
                if (gamepad1.left_trigger != 0) {
                    robotSpeed = baseSpeed * 1.4;
                } else if (gamepad1.right_trigger != 0) {
                    robotSpeed = baseSpeed * .4;
                } else {
                    robotSpeed = baseSpeed;
                }
                if (gamepad1.a) {
                    //Button A = unconstrained movement
                    constrainMovement = false;
                }
                if (gamepad1.b) {
                    //Button B = constrained movement
                    constrainMovement = true;
                    // Flip the boolean to toggle modes for drive contraints
                    //constrainMovement = !constrainMovement;
                }
                // We cubed the inputs to make the inputs more responsive
                y = Math.pow(gamepad1.left_stick_y, 3); // Remember, this is reversed!
                x = Math.pow(gamepad1.left_stick_x * -1.1, 3); // Counteract imperfect strafing
                rx = Math.pow(gamepad1.right_stick_x, 3) * 0.5;  //Reduced turn speed to make it easier to control

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                if (constrainMovement) {
                    if (x > y) {
                        leftFrontPower = (x - rx) / denominator;
                        leftRearPower = (-x - rx) / denominator;
                        rightFrontPower = (-x + rx) / denominator;
                        rightRearPower = (x + rx) / denominator;
                    }
                    if (y > x) {
                        leftFrontPower = (y - rx) / denominator;
                        leftRearPower = (y - rx) / denominator;
                        rightFrontPower = (y + rx) / denominator;
                        rightRearPower = (y + rx) / denominator;
                    }
                } else {
                    leftFrontPower = (y + x - rx) / denominator;
                    leftRearPower = (y - x - rx) / denominator;
                    rightFrontPower = (y - x + rx) / denominator;
                    rightRearPower = (y + x + rx) / denominator;
                }


                if (gamepad2.dpad_up) {
                    robot.armMotor.setPower(-.6);

                } else  if (gamepad2.dpad_down) {


                    robot.armMotor.setPower(.4);

                }


                Log.i("VVTeleOpPWP", String.valueOf(gamepad2.left_stick_y));

                //set arm with analog stick
                if (gamepad2.left_stick_y !=0 )
                {
                   
                    if ((Math.abs(currentEncoderCount) < Math.abs(maxEncoderCount) ) && (gamepad2.left_stick_y  < 0)  )
                    {
                        robot.armMotor.setPower(gamepad2.left_stick_y * 1);

                    }
                    else
                    {
                        robot.armMotor.setPower(gamepad2.left_stick_y );
                    }
                }
                else
                {
                    //code to prevent the Arm coming crashing down
                    robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    robot.armMotor.setPower(0);

                }


                robot.setWheelsSpeed(leftFrontPower * robotSpeed*.95, -rightFrontPower * robotSpeed*.95,
                        (leftRearPower * robotSpeed*.94), (-rightRearPower * robotSpeed*.94));

            }
            // End gamepad 1
        } catch (Exception e) {
            telemetry.addData(">",e.toString());
            telemetry.update();
            e.printStackTrace();
        }

    }



}




