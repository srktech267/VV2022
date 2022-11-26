package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.VVWebcamAutonTS;
import org.firstinspires.ftc.teamcode.core.VVRobot;
import org.firstinspires.ftc.teamcode.core.VVRobotOps;

@Autonomous(name="AutonImageDetecter" , group="AutoTestVV2022")
@Disabled
public class AutonImageDetecter extends VVWebcamAutonTS {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    VVRobotOps robotOps = null;



    //servo

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    public Servo servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;




    @Override
    public void runOpMode() {

        //servo end
        robot = new VVRobot(hardwareMap);

        robotOps = new VVRobotOps();
        //Start Image detection
        // create an instance of the image detection Thread.
        detectSignal();
        servo = robot.getConePickupServo();
        servo.getController().pwmEnable();



        // Display the current value
        telemetry.addData("slingImage", slingImage);
        telemetry.update();
        robotOps.stop();
        Log.i("VVTensor", "before waitForStart() ==> "+ opModeIsActive());
        waitForStart();
        clawClose();

        Log.i("VVTensor", "slingImage  ==> "+ slingImage);
        Log.i("VVTensor", "before waitForStart() isStarted  ==> "+ isStarted());

        if (opModeIsActive()) {
            detectSignalFlag = false;

            Log.i("VVTensor", "slingImage  ==> "+ slingImage);
            Log.i("VVTensor", "after waitForStart() isStarted  ==> "+ isStarted());


            // Display the current value
            telemetry.addData("slingImage", slingImage);
            telemetry.update();
            loadCone();


        }
        robotOps.stop();

    }

    public void loadCone()
    {

        robot.armMotor.setPower(-.3);
        vvSleep(1.3);
        clawOpen();
        vvSleep(2);
        clawClose();
        vvSleep(1);

        robot.armMotor.setPower(-.1);
        vvSleep(2);
    }

    public void clawOpen( ) {
        // Set the servo to the new position and pause;
        //open
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(1);
    }
    public void clawClose( ) {
        // Set the servo to the new position and pause;
        //close
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(1);
    }

    public void vvSleep(double sleepTime)
    {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<sleepTime)
        {

        }
    }
}
