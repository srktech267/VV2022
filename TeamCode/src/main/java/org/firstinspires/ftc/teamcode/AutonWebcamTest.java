package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonWebcamTest" , group="AutoTestVV2022")
@Disabled
public class AutonWebcamTest extends VVWebcamAuton{
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



        if (opModeIsActive()) {
           detectSignalFlag = false;

            Log.i("VVTensor", "after waitForStart() detectSignalFlag  ==> "+ this.detectSignalFlag);

            // Display the current value
            telemetry.addData("slingImage", slingImage);
            telemetry.update();
            clawClose();
            loadCone();


   if (slingImage != null) {
       if (slingImage.equals("3 Panel")) {
           robotOps.driveFront(robot, .3, 3000);
           robotOps.driveBack(robot, .3, 1500);
           robotOps.slideRight(robot, .3, 2800);
       } else if (slingImage.equals("2 Bulb")) {
           robotOps.driveFront(robot, .4, 2000);
       } else if (slingImage.equals("1 Bolt")) {
           robotOps.driveFront(robot, .4, 2000);
           robotOps.slideLeft(robot, .3, 2000);
       }
   }
       else
       {
           telemetry.addData("slingImage", "Not found");
           telemetry.update();
       }



        }
        robotOps.stop();

    }

    public void loadCone()
    {

        robot.armMotor.setPower(-.3);
        sleep(1300);
        robotOps.driveFront(robot,.2,300);
        robotOps.slideLeft(robot,.3,1200);
        robotOps.driveFront(robot,.2,500);
        robot.armMotor.setPower(-.2);
        clawOpen();
        sleep(2000);
        robotOps.driveBack(robot,.2,500);
        robotOps.slideRight(robot,.3,875);
    }

    public void clawOpen( ) {
        // Set the servo to the new position and pause;
        //open
        Log.i("VVServo", "Before open : "+servo.getPosition());
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(2);
        servo.setDirection(Servo.Direction.FORWARD);
        Log.i("VVServo", "after open : "+servo.getPosition());

        }
    public void clawClose( ) {
        // Set the servo to the new position and pause;
        //close
        servo.getController().pwmEnable();
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(1);
        sleep(1000);

    }
}

        
