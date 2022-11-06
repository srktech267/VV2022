package org.firstinspires.ftc.teamcode;

import android.view.Display;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonComp6" , group="AutoTestVV2022")
@Disabled
public class AutonComp6 extends LinearOpMode {
// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();




    //servo

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    public Servo servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;





    public void runOpMode() {

        //servo end
        VVRobot robot = new VVRobot(hardwareMap);

        VVRobotOps robotOps = new VVRobotOps();

        servo = robot.getConePickupServo();


        waitForStart();
        runtime.reset();
        robotOps.stop();
        if (opModeIsActive()) {



            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

        clawclose();
        robot.armMotor.setPower(-.3);
        sleep(1000);
        robotOps.slideLeft(robot,.3,1500);
        robotOps.driveFront(robot,.2,875);
        robot.armMotor.setPower(-.2);
        sleep(2000);
        clawOpen();
        robotOps.driveBack(robot,.2,1000);
        clawclose();
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armMotor.setPower(-.1);
       // sleep(2000);
        robotOps.slideLeft(robot,.3,1200);
        robotOps.driveFront(robot,.3,1750);
        robotOps.turn(robot,VVConstants.Right,VVConstants.DEGREES_90);
        telemetry.addData("arm motor power", robot.armMotor.getZeroPowerBehavior());

          telemetry.update();



        }
        robotOps.stop();

    }

    public void clawOpen( ) {


            // Set the servo to the new position and pause;
            //close
            servo.getController().pwmEnable();
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(.5);
            sleep(2000);

        }
    public void clawclose( ) {
        // Set the servo to the new position and pause;
        //open
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0);
        sleep(1000);

    }
}

        
