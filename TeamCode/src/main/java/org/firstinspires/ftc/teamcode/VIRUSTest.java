package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by pbai on 11/4/2016.
 */
@TeleOp(name="VIRUSTest", group="OpMode")

public class VIRUSTest extends OpMode{
    DcMotor rMotor0;
    DcMotor lMotor0;
    DcMotor rMotor1;
    DcMotor lMotor1;
    ModernRoboticsI2cGyro   gyro;
    FrameGrabber frameGrabber;

    final static double COUNTS_PER_ROTATION = 280;
    final static double WHEEL_CIRCUM = Math.PI * 4.0;
    final static double GEAR_RATIO = 2.0 / 3.0;
    final static double COUNTS_PER_INCH = COUNTS_PER_ROTATION * GEAR_RATIO /WHEEL_CIRCUM;
    double startValue;

    final static double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    final static double DRIVE_COEFF = 0.05;

    enum state {testTurn, testForward, GetBeaconColor,stop}
    state state;


    @Override
    public void init() {
        lMotor0 = hardwareMap.dcMotor.get("lmotor0");
        rMotor0 = hardwareMap.dcMotor.get("rmotor0");
        lMotor1 = hardwareMap.dcMotor.get("lmotor1");
        rMotor1 = hardwareMap.dcMotor.get("rmotor1");
        gyro=(ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gsensor");

        lMotor0.setDirection(DcMotor.Direction.REVERSE);
        lMotor1.setDirection(DcMotor.Direction.REVERSE);

        lMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {

        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();
    }

    public void start() {
        lMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyro.resetZAxisIntegrator();
        frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get theframeGrabber
        frameGrabber.grabSingleFrame(); //Tell it to grab a frame
        state = state.GetBeaconColor;
    }

    public void loop() {
        switch (state) {
            case testTurn:
                if (onHeading(TURN_SPEED, -45.0, P_TURN_COEFF)) {
                    startValue = gyro.getIntegratedZValue();
                    driveForwardInit(10);
                    state = state.testForward;
                }
                telemetry.update();
                break;

            case testForward:
                if (distanceReached()==true){
                    state = state.stop;
                }
                break;
            case GetBeaconColor:

                 if(frameGrabber.isResultReady()) { //Wait for the result
                     //Get the result
                     ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
                     BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();
                     BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
                     BeaconColorResult.BeaconColor rightColor = result.getRightColor();
                     telemetry.addData("Result", result); //Display it on telemetry
                     telemetry.update();
                 }
                break;
            case stop:
                lMotor0.setPower(0);
                rMotor0.setPower(0);
                lMotor1.setPower(0);
                rMotor1.setPower(0);
                break;

        }

    }
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        lMotor0.setPower(leftSpeed);
        rMotor0.setPower(rightSpeed);
        lMotor1.setPower(leftSpeed);
        rMotor1.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void driveForwardInit(double Distance){
        int target =(int) (Distance * COUNTS_PER_INCH);

        lMotor0.setTargetPosition( lMotor0.getCurrentPosition()+target);
        rMotor1.setTargetPosition( lMotor1.getCurrentPosition()+target);
        lMotor0.setTargetPosition ( rMotor0.getCurrentPosition()+target);
        rMotor1.setTargetPosition( rMotor1.getCurrentPosition()+target);
        lMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    boolean distanceReached () {
        boolean reached = false;
        double difference=DRIVE_COEFF*(gyro.getIntegratedZValue()-startValue);
        double lSpeed = 0.5-difference;
        double rSpeed = 0.5+difference;
        double max = Math.max(Math.abs(lSpeed), Math.abs(rSpeed));
        if (max > 1.0)
        {
            lSpeed /= max;
            rSpeed /= max;
        }

        lMotor0.setPower(lSpeed);
        rMotor0.setPower(rSpeed);
        lMotor1.setPower(lSpeed);
        rMotor1.setPower(rSpeed);
        if (lMotor0.isBusy() == false && lMotor1.isBusy() == false && rMotor1.isBusy() == false && rMotor0.isBusy() == false) {
            return reached = true;
        }
        return reached;
    }






}




