package org.firstinspires.ftc.teamcode.backup;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.VVWebcamAutonTS;
import org.firstinspires.ftc.teamcode.core.VVRobot;
import org.firstinspires.ftc.teamcode.core.VVRobotOps;

@Autonomous(name="AutonCompRedV1" , group="AutonVV2022")
@Disabled
public class AutonCompRedV1 extends VVWebcamAutonTS {
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
        servo.getController().pwmEnable();

        waitForStart();

        if (opModeIsActive()) {
            detectSignalFlag = false;

            Log.i("VVTensor", "after waitForStart() detectSignalFlag  ==> "+ this.detectSignalFlag);

            // Display the current value
            telemetry.addData("slingImage", slingImage);
            telemetry.update();
            loadCone();

            if (slingImage != null) {
                if (slingImage.equals("3 Panel")) {

                    robotOps.driveFront(robot, .3, 3000);
                    robotOps.driveBack(robot, .3, 1200);
                    robotOps.slideLeft(robot, .3, 1900);

                } else if (slingImage.equals("2 Bulb")) {
                    robotOps.driveFront(robot, .3, 3000);
                    robotOps.driveBack(robot, .3, 1200);

                } else if (slingImage.equals("1 Bolt")) {

                    robotOps.driveFront(robot, .3, 3000);
                    robotOps.driveBack(robot, .3, 1200);
                    robotOps.slideRight(robot, .3, 1900);
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
        clawClose();
        robot.armMotor.setPower(-.3);
        vvSleep(1.75);
        robotOps.driveFront(robot,.2,280);
        robotOps.slideRight(robot,.3,1500);
        robotOps.driveFront(robot,.2,600);
        robot.armMotor.setPower(-.2);
        vvSleep(2);
        clawOpen();
        vvSleep(2);

        robotOps.driveBack(robot,.2,600);
        clawClose();
        robot.armMotor.setPower(.2);
        robotOps.slideLeft(robot,.3,1500);

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
        vvSleep(.5);
    }

    public void vvSleep(double sleepTime)
    {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds()<sleepTime)
        {

        }
    }
}
