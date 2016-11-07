package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by pbai on 10/15/2016.
 */
@TeleOp(name="RAutonomous", group="LinearOpMode")
public class RAutonomous extends LinearOpMode{
    DcMotor rmotor0;
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    GyroSensor gsensor;
    final static int encodercnt=280;
    final static double circ = Math.PI*4.0;
    final static double gearRatio=2/3;


    @Override
    public void runOpMode() throws InterruptedException {
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        gsensor = hardwareMap.gyroSensor.get("gsensor");

        lmotor0.setDirection(DcMotor.Direction.REVERSE);
        lmotor1.setDirection(DcMotor.Direction.REVERSE);

        lmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gsensor.calibrate();
        while (gsensor.rawZ() >= 45){
            rmotor0.setPower(1);
            lmotor0.setPower(-1);
            rmotor1.setPower(1);
            lmotor1.setPower(-1);
        }
        lmotor0.setPower(0);
        rmotor0.setPower(0);
        lmotor1.setPower(0);
        rmotor1.setPower(0);
        while (lmotor0.getCurrentPosition()<=(10/circ)*encodercnt*gearRatio){
            lmotor0.setPower(1);
            rmotor0.setPower(1);
            lmotor1.setPower(1);
            rmotor1.setPower(1);
        }
        lmotor0.setPower(0);
        rmotor0.setPower(0);
        lmotor1.setPower(0);
        rmotor1.setPower(0);
    }
}
