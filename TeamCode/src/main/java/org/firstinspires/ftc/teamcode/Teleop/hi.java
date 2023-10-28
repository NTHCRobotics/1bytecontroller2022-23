
package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
/*
    @Autonomous = this is for Autonomous mode
    @TeleOp = this is for User Controlled mode

    name = the name that will display on the Driver Hub
    group = allows you to group OpModes
 */
@TeleOp(name="thefootbll)", group="sai")

//@Disabled  This way it will run on the robot
public class hi extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();  //timer

    /*
    Declare motors to type DcMotorEx

    Documentation:
    https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
     */

    //Touch Sensors
    //private DigitalChannel intakeSensor;

    //Motors
    private Rev2mDistanceSensor sideLeftDistanceSensor;
    private Rev2mDistanceSensor sideRightDistanceSensor;
    private DcMotorEx Left;
    private DcMotorSimple; // this would be the righr side motor that controls the effectiveness of the lazy susan or turntabl





    //private DcMotorEx Insertnamehere
    //private DcMotorEx Insertnamehere


    //Servos
    private CRServo camera;











    //double susanPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {



        telemetry.addData("Status", "Initialization Started");


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //Motors
        Left= hardwareMap.get(DcMotorEx.class, "Left");
        Left = hardwareMap.get(DcMotorEx.class, "Right");


        //Motor Encoders
        //Wheels
        Left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Left.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sideLeftDistanceSensor()

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete");



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//this will run the methods repeadtly
        wepon();







//        telemetry.addData("range", String.format("%.3f cm", sideDistanceSensor.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range edited", sideDistanceSensor.getDistance(DistanceUnit.CM));

        telemetry.update();
    }


    public void wepon() {
        if (gamepad1.left_trigger > 0) {
            Left.setVelocity(2000);
            Right.setVelocity(-2000);
            gamepad1.rumble(0.7, 0.7, 200);
        } else if (gamepad1.right_trigger > 0) {

            Left.setVelocity(-2000);
            Right.setVelocity(2000);
            gamepad1.rumble(0.7, 0.7, 200);

        } else {
            Left.setVelocity(0);
            Right.setVelocity(0);

            //youtube
        }
    }}



        /*
         * Code to run ONCE after the driver hits STOP
         */

        /*
         * Code to run ONCE after the driver hits STOP
         */


    //@Override

