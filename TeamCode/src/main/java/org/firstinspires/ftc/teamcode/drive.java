package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by mzhang on 10/1/2016.
 */
@TeleOp(name="drive", group="TeleOp")


public class drive extends OpMode {
    DcMotor rmotor0;
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    DcMotor sweeper;
    DcMotor shooter0;
    DcMotor shooter1;

    BeaconColorResult Result;

    @Override
    public void init(){
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter0 = hardwareMap.dcMotor.get("shooter0");
        shooter1 = hardwareMap.dcMotor.get("shooter1");

        rmotor0.setDirection(DcMotor.Direction.REVERSE);
        rmotor1.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    public void loop(){
        float lefty= -gamepad1.left_stick_y;
        float leftx= -gamepad1.left_stick_x;
        float righty= -gamepad1.right_stick_y;
        float rightx= -gamepad1.right_stick_x;
        float rtrigger= -gamepad1.right_trigger;
        float ltrigger= -gamepad1.left_trigger;
       if (rightx==0&&rtrigger==0&&ltrigger==0){
            rmotor0.setPower(-lefty);
            lmotor0.setPower(-lefty);
            rmotor1.setPower(-lefty);
            lmotor1.setPower(-lefty);

        }

        if (lefty==0&&rtrigger==0&&ltrigger==0) {
            rmotor0.setPower(-rightx);
        lmotor0.setPower(rightx);
        rmotor1.setPower(rightx);
        lmotor1.setPower(-rightx);
        }
      if (lefty==0&&rightx==0&&rtrigger==0&&ltrigger==0){
            rmotor0.setPower(0);
            lmotor0.setPower(0);
            rmotor1.setPower(0);
            lmotor1.setPower(0);
        }
        if (lefty==0&&rightx==0&&ltrigger==0){
            rmotor0.setPower(rtrigger);
            rmotor1.setPower(rtrigger);
            lmotor0.setPower(-rtrigger);
            lmotor1.setPower(-rtrigger);

        }
        if (lefty==0&&rightx==0&&rtrigger==0){
            rmotor0.setPower(-ltrigger);
            rmotor1.setPower(-ltrigger);
            lmotor0.setPower(ltrigger);
            lmotor1.setPower(ltrigger);

        }
        if (gamepad1.right_bumper==true&&gamepad1.left_bumper==false){
            sweeper.setPower(1);
        }
        else if(gamepad1.left_bumper==true&&gamepad1.right_bumper==false){
            sweeper.setPower(-1);
        }
        else if (gamepad1.left_bumper==false&&gamepad1.right_bumper==false){
            sweeper.setPower(0);
        }
        if (gamepad1.start==true){
            shooter0.setPower(-1);
            shooter1.setPower(1);

        }
        if (gamepad1.start==false){
            shooter0.setPower(0);
            shooter1.setPower(0);

        }

    }
}

