package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonComp5" , group="AutoTestVV2022")

public class AutonComp5 extends LinearOpMode {
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
    boolean rampUp = true;

    //servo end


    public void runOpMode() {


        VVRobot robot = new VVRobot(hardwareMap);

        VVRobotOps robotOps = new VVRobotOps();

        VVClaw claw = new VVClaw(robot.getConePickupServo(), telemetry);
        Servo servo = robot.getConePickupServo();

        waitForStart();
        runtime.reset();
        robotOps.stop();
        if (opModeIsActive()) {



                robotOps.slideLeft(robot,.5,1000);


                // Display the current value
                telemetry.addData("Servo Position", "%5.2f", position);
                telemetry.addData(">", "Press Stop to end test.");
                telemetry.update();





       // claw.closeClaw();
        telemetry.addData("closeClaw", "closeClaw");
        telemetry.update();


        telemetry.addData("claw Position", claw.vvServo.getPosition());

          telemetry.update();
            telemetry.log();
            sleep(10000);
        }
        robotOps.stop();

    }



        
        
}
