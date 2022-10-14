package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autondemo" , group="AutoTestVV2022")

public class Autondemo extends LinearOpMode {
// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DistanceSensor sensorRange;
    double ringHeight;
    DistanceSensor distance;
    
    
    public void runOpMode() {


        VVRobot robot = new VVRobot(hardwareMap);

        VVRobotOps robotOps = new VVRobotOps();

        waitForStart();


        runtime.reset();

        //robotOps.slideLeft( robot, .5, 3000);
        //robotOps.turn(robot,.45, 45);
        robotOps.driveFront(robot,.3,5000);

        sleep(5000);

        runtime.reset();



        
        robotOps.stop();

    }



        
        
}
