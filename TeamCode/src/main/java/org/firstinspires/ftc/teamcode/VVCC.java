package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "OpMode",name = "VVOp")
public class VVCC extends  OpMode
{


    private DcMotor FLM;
    private DcMotor FRM;
    private DcMotor LMB;
    private DcMotor RMB;

    private double power;

    //we created an object of ElapsedTime
    private ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void init() {

          //the assigned names should to same as names mapped to BOT.
          FLM = hardwareMap.dcMotor.get("LMF");
          FRM = hardwareMap.dcMotor.get("FRM");
          LMB = hardwareMap.dcMotor.get("LMB");
          RMB = hardwareMap.dcMotor.get("RMB");

        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FLM.setPower(0.0);
        FRM.setPower(0.0);
        LMB.setPower(0.0);
        RMB.setPower(0.0);

        telemetry.addData("status","Initialized");
        telemetry.update();



    }

    @Override
    public void loop() {

    }


}
