package org.firstinspires.ftc.teamcode.test;


import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp(name="clawtester" , group="TesterVV2022")
@Disabled
public class VVClawTest extends LinearOpMode {


    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    static final double Servo_start_POS = 0.0;
    static final double Servo_end_POS = 1.0;


    double position = 0;
    boolean rampUp = true;


    CRServoImpl  servo = null;


    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.

        Log.i("clawtest","in runOpMode()");
        try {
             servo = (CRServoImpl) hardwareMap.get(CRServo.class, "CPM");


            servo.resetDeviceConfigurationForOpMode();
        } catch (Exception e) {
        Log.i("clawtest",e.toString());
        e.printStackTrace();
    }

        waitForStart();
        try {
        while (opModeIsActive()) {

            if (gamepad2.left_bumper)
            {
                servo.setPower(128);
                sleep(500);

                servo.setPower(0);
            }
            else if  (gamepad2.right_bumper)
            {
               // servo.setDirection(DcMotorSimple.Direction.REVERSE);
                servo.setPower(-.4);
                sleep(500);
                servo.setPower(0);
            }




        }
        } catch (Exception e) {
            Log.i("clawtest",e.toString());
            e.printStackTrace();
        }


    }
}





