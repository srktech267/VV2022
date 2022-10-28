package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonComp6b" , group="AutoTestVV2022")

public class AutonComp6b extends LinearOpMode {
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
        sleep(1300);
        robotOps.driveFront(robot,.2,300);
        robotOps.slideRight(robot,.3,1750);
        robotOps.driveFront(robot,.2,500);
        robot.armMotor.setPower(-.2);
        sleep(2000);
        clawOpen();
        robotOps.driveBack(robot,.2,700);
        clawclose();
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("arm motor power", robot.armMotor.getZeroPowerBehavior());
        telemetry.update();

        robot.armMotor.setPower(-.1);
       // sleep(2000);
        robotOps.slideRight(robot,.3,1200);
        robotOps.driveBack(robot,.3,100);
        robotOps.turn(robot,VVConstants.Right,VVConstants.DEGREES_20);
        robotOps.driveFront(robot,.3,3250);
        robotOps.turn(robot,VVConstants.Left,VVConstants.DEGREES_100);
        robotOps.driveFront(robot,.3,3000);

            clawOpen();
            robot.armMotor.setPower(-.275);
            sleep(1000);
            clawclose();
            robotOps.driveBack(robot,.2,1500);






        }
        robotOps.stop();

    }

    public void clawOpen( ) {


            // Set the servo to the new position and pause;
            //close
            servo.getController().pwmEnable();
            servo.setDirection(Servo.Direction.FORWARD);
            servo.setPosition(.5);
            sleep(1000);

        }
    public void clawclose( ) {
        // Set the servo to the new position and pause;
        //open
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0);
        sleep(1000);

    }
}

        
