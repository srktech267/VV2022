package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.VVRobot;
import org.firstinspires.ftc.teamcode.core.VVRobotOps;

@Autonomous(name="AutonTestIMU" , group="AutoTestVV2022")
@Disabled
public class AutonTestIMU extends LinearOpMode {
// Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();




    public void runOpMode() {


        VVRobot robot = new VVRobot(hardwareMap);

        VVRobotOps robotOps = new VVRobotOps();

        VVClaw claw = new VVClaw(robot.getConePickupServo(), telemetry);

        waitForStart();
        runtime.reset();
        robotOps.stop();
        if (opModeIsActive()) {


             VVRobotTurn VT = new VVRobotTurn(robot,robotOps);


        }
        robotOps.stop();

    }



        
        
}
