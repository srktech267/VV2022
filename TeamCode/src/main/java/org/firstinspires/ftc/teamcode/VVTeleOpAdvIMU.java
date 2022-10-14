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




import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="Pushbot: VVTeleOpAdvIMU123", group="Pushbot")

public class VVTeleOpAdvIMU extends LinearOpMode {

    /* Declare OpMode members. */
    VVRobot robot = null;

    float rotate_angle = 0;
    double reset_angle = 0;
    BNO055IMU imu;
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
        robot = new VVRobot(hardwareMap, "IMU", true);
        imu =  robot.imu;


            while(!opModeIsActive()){}

            while(opModeIsActive()){                driveWithIMU();
                resetAngle();
                //driveSimple();
                telemetry.update();
            }
        }
        public void driveSimple(){
            double power = .5;
            if(gamepad1.dpad_up){ //Forward
                robot.front_left_wheel.setPower(-power);
                robot.back_left_wheel.setPower(-power);
                robot.back_right_wheel.setPower(-power);
                robot.front_right_wheel.setPower(-power);
            }
            else if(gamepad1.dpad_left){ //Left
                robot.front_left_wheel.setPower(power);
                robot.back_left_wheel.setPower(-power);
                robot.back_right_wheel.setPower(power);
                robot.front_right_wheel.setPower(-power);
            }
            else if(gamepad1.dpad_down){ //Back
                robot.front_left_wheel.setPower(power);
                robot.back_left_wheel.setPower(power);
                robot.back_right_wheel.setPower(power);
                robot.front_right_wheel.setPower(power);
            }
            else if(gamepad1.dpad_right){ //Right
                robot.front_left_wheel.setPower(-power);
                robot.back_left_wheel.setPower(power);
                robot.back_right_wheel.setPower(-power);
                robot.front_right_wheel.setPower(power);
            }
            else if(Math.abs(gamepad1.right_stick_x) > 0){ //Rotation
                robot.front_left_wheel.setPower(-gamepad1.right_stick_x);
                robot.back_left_wheel.setPower(-gamepad1.right_stick_x);
                robot.back_right_wheel.setPower(gamepad1.right_stick_x);
                robot.front_right_wheel.setPower(gamepad1.right_stick_x);
            }
            else{
                robot.front_left_wheel.setPower(0);
                robot.back_left_wheel.setPower(0);
                robot.back_right_wheel.setPower(0);
                robot.front_right_wheel.setPower(0);
            }
        }

    public void driveAdv()
    {

    }
        public void driveWithIMU()
        {
            double Protate = gamepad1.right_stick_x/4;
            double stick_x = gamepad1.left_stick_x * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2); //Accounts for Protate when limiting magnitude to be less than 1
            double stick_y = gamepad1.left_stick_y * Math.sqrt(Math.pow(1-Math.abs(Protate), 2)/2);
            double theta = 0;
            double Px = 0;
            double Py = 0;

            double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
            if (gyroAngle <= 0) {
                gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
                gyroAngle = gyroAngle + (Math.PI / 2);
            } else if (Math.PI / 2 <= gyroAngle) {
                gyroAngle = gyroAngle - (3 * Math.PI / 2);
            }
            gyroAngle = -1 * gyroAngle;

            if(gamepad1.right_bumper){ //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
                gyroAngle = -Math.PI/2;
            }

            //Linear directions in case you want to do straight lines.
            if(gamepad1.dpad_right){
                stick_x = 0.5;
            }
            else if(gamepad1.dpad_left){
                stick_x = -0.5;
            }
            if(gamepad1.dpad_up){
                stick_y = -0.5;
            }
            else if(gamepad1.dpad_down){
                stick_y = 0.5;
            }


            //MOVEMENT
            theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
            Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
            Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));

            telemetry.addData("Stick_X", stick_x);
            telemetry.addData("Stick_Y", stick_y);
            telemetry.addData("Magnitude",  Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)));
            telemetry.addData("Front Left", Py - Protate);
            telemetry.addData("Back Left", Px - Protate);
            telemetry.addData("Back Right", Py + Protate);
            telemetry.addData("Front Right", Px + Protate);



            robot.front_left_wheel.setPower(Py - Protate);
            robot.back_left_wheel.setPower(Px - Protate);
            robot.back_right_wheel.setPower(Py + Protate);
            robot.front_right_wheel.setPower(Px + Protate);
        }
        public void resetAngle(){
            if(gamepad1.a){
                reset_angle = getHeading() + reset_angle;
            }
        }
        public double getHeading(){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
            double heading = angles.firstAngle;
            if(heading < -180) {
                heading = heading + 360;
            }
            else if(heading > 180){
                heading = heading - 360;
            }
            heading = heading - reset_angle;
            return heading;
        }
}




