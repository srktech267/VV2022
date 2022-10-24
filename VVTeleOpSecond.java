package org.firstinspires.ftc.teamcode;


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



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="VVTeleOpSecond", group="Tele-op")

public class VVTeleOpSecond extends LinearOpMode {


    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;                   // sets rate to move servo
    private ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Initialized");
        VVRobot robot = new VVRobot(hardwareMap);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // based Mech setup and gear positions , FL and BL should have opposite polarity / speed/ direction
        //FL,FR
        //BL,BR
        //1,2
        //3,4
        //-,+
        //-,+

        // run until the end of the match (driver presses STOP)

        robot.setWheelsAllForward();
        while (opModeIsActive()) {


                double power = .5;

                //DPAD code begins
                if(gamepad1.dpad_up){ //Forward
                    //FL,FR, BL,BR

                   robot.setWheelsSpeed(-power,power,-power,power);

                }
                else if(gamepad1.dpad_left){ //Left
                    //FL,FR, BL,BR
                    robot.setWheelsSpeed(power,power,-power,-power);
                }
                else if(gamepad1.dpad_down){ //Back

                    //FL,FR, BL,BR
                    robot.setWheelsSpeed(power,-power,power,-power);
                }
                else if(gamepad1.dpad_right){ //Right
                    //FL,FR, BL,BR
                    robot.setWheelsSpeed(-power,-power,power,power);

                }


                // Analog Stick code begins

                else if(Math.abs(gamepad1.right_stick_y) > 0){ //Rotation
                    //FL,FR, BL,BR
                    robot.setWheelsSpeed(0,-power,0,-power);
                }

                else if(Math.abs(gamepad1.left_stick_y) > 0){ //Rotation
                    //FL,FR, BL,BR
                    robot.setWheelsSpeed(-power,0,-power,0);
                }







                else{
                    robot.front_left_wheel.setPower(0);
                    robot.back_left_wheel.setPower(0);
                    robot.back_right_wheel.setPower(0);
                    robot.front_right_wheel.setPower(0);
                }


            }




        }
    }





