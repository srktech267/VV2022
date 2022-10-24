package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware. Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VVClaw   {


    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    static final double Servo_start_POS = 0.0;
    static final double Servo_end_POS = 1.0;


    double position = 0;
    boolean rampUp = true;

    public Servo getVvServo() {
        return vvServo;
    }

    Servo vvServo = null;
    Telemetry telemetry = null;


    public VVClaw( Servo vvs, Telemetry tm) {
        vvServo = vvs ;
        telemetry = tm;

    }



    public void openClaw()
    {




        // Scan servo till stop pressed.
        while(true){

            // slew the servo, according to the rampUp (direction) variable.


            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                if (position >= MAX_POS ) {

                    break;
                }
            }


            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            vvServo.setPosition(position);
           // sleep(CYCLE_MS);
           // idle();
        }
        }
    public void closeClaw() {

        // Scan servo till stop pressed.
        while(true){

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position -= INCREMENT ;
                if (position >= MAX_POS ) {

                    break;
                }
            }


            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            vvServo.setPosition(position);
           // sleep(CYCLE_MS);
          //  idle();
        }


    }



}





