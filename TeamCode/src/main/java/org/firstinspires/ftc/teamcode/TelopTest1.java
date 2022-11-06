package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TelopTest1" , group="TelopTestGroup")
@Disabled
public class TelopTest1 extends LinearOpMode {
// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();




    //servo

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean clawRelease = true;

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Initialized");
        VVRobot robot = new VVRobot(hardwareMap);


        VVRobotOps robotOps = new VVRobotOps();


        Servo servo = robot.getConePickupServo();


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

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.addLine();
            telemetry.addData("direction", servo.getDirection());
            telemetry.update();

            if (gamepad2.y)
            {

                // Set the servo to the new position and pause;
                //close
                servo.getController().pwmEnable();
                servo.setDirection(Servo.Direction.FORWARD);
                servo.setPosition(1);
                sleep(1000);
                clawRelease = false;
            }
            else if  (gamepad2.b)
            {

                // Set the servo to the new position and pause;
                //open
                servo.setDirection(Servo.Direction.REVERSE);
                servo.setPosition(.6);
                sleep(1000);
                servo.getController().pwmDisable();
            }








        }




    }



        
        
}
