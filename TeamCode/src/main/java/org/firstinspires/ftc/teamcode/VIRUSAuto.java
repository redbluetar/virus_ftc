package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by pbai on 10/15/2016.
 */
@TeleOp(name="VIRUSAuto", group="OpMode")
public class VIRUSAuto extends OpMode {
    DcMotor rmotor0;
    DcMotor lmotor0;
    DcMotor rmotor1;
    DcMotor lmotor1;
    GyroSensor gsensor;
    final static int encodercnt = 280;
    final static double circ = Math.PI * 4.0;
    final static double gearRatio = 2.0 / 3.0;
    final static double cntperin = encodercnt*gearRatio/circ;

    enum state {testTurn, testForward, GetBeaconColor, stop}
     state state;

    @Override
    public void init() {
        lmotor0 = hardwareMap.dcMotor.get("lmotor0");
        rmotor0 = hardwareMap.dcMotor.get("rmotor0");
        lmotor1 = hardwareMap.dcMotor.get("lmotor1");
        rmotor1 = hardwareMap.dcMotor.get("rmotor1");
        gsensor = hardwareMap.gyroSensor.get("gsensor");

        lmotor0.setDirection(DcMotor.Direction.REVERSE);
        lmotor1.setDirection(DcMotor.Direction.REVERSE);
        gsensor.calibrate();
        lmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {
        lmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lmotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        state = state.testForward;
    }

    public void loop() {
        switch (state) {
            case testTurn:
                rmotor0.setPower(1);
                lmotor0.setPower(-1);
                rmotor1.setPower(1);
                lmotor1.setPower(-1);
                if (gsensor.rawZ() >= 45) {
                    rmotor0.setPower(0);
                    lmotor0.setPower(0);
                    rmotor1.setPower(0);
                    lmotor1.setPower(0);
                    state = state.testForward;
                }

                break;
            case testForward:
                lmotor0.setPower(1);
                rmotor0.setPower(1);
                lmotor1.setPower(1);
                rmotor1.setPower(1);
                telemetry.addData("circ", circ);
                telemetry.addData("cntperin", cntperin);
                telemetry.addData("gearRatio", gearRatio);

                telemetry.addData("encoder", lmotor0.getCurrentPosition());
                if (lmotor0.getCurrentPosition() >= (48*cntperin)){
                    lmotor0.setPower(0);
                    rmotor0.setPower(0);
                    lmotor1.setPower(0);
                    rmotor1.setPower(0);
                    telemetry.addData("stop", 1);
                    state = state.stop;

                }

                break;

            case GetBeaconColor:

                break;
            case stop:
                lmotor0.setPower(0);
                rmotor0.setPower(0);
                lmotor1.setPower(0);
                rmotor1.setPower(0);
                break;






        }
    }
}
