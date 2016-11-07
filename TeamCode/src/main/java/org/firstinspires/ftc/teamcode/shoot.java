package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mzhang on 10/1/2016.
 */
@TeleOp(name="Shooter", group="TeleOp")


public class shoot extends OpMode {
    DcMotor rmotor;
    DcMotor lmotor;

    @Override
    public void init(){
        lmotor = hardwareMap.dcMotor.get("lmotor");
        rmotor = hardwareMap.dcMotor.get("rmotor");
    }
    @Override
    public void loop(){
        rmotor.setPower(1);
        lmotor.setPower(-1);
    }
}

