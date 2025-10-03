package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartPoint;
import com.github.bouyio.cyancore.localization.TankKinematics;
import com.github.bouyio.cyancore.localization.ThreeDeadWheelOdometry;
import com.github.bouyio.cyancore.util.Distance;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * <p>
 *     This class demonstrates three dead wheel localization in CyanFTC using kinematic formulas.
 * </p>
 * */
@Disabled
@TeleOp()
public class ThreeDeadWheelLocalization extends OpMode {

    // TODO: Set constants to match the ones of the robot
    // NOTE: The distance unit of the track width must be
    // the same as the distance unit of the converted motor
    // rotations.
    final double ENCODER_WIDTH = 0000000000;
    final double ENCODER_WHEEL_RADIUS = 000000000;
    final double ENCODER_COUNT_PER_REVOLUTION = 000000000;
    final double TICKS_TO_LINEAR_DISTANCE = 2 * Math.PI * ENCODER_WHEEL_RADIUS / ENCODER_COUNT_PER_REVOLUTION;


    // TODO: Set the init position coordinates and heading.
    double INITIAL_POS_X = 0000000000;
    double INITIAL_POS_Y = 0000000000;
    double INITIAL_HEADING = 000000000;


    /**The localization system.*/
    ThreeDeadWheelOdometry odometry;

    // The encoders we will be using.
    DcMotor leftParallelEncoder;
    DcMotor rightParallelEncoder;
    DcMotor perpendicularEncoder;

    /**The logging system.*/
    Logger logger;

    @Override
    public void init() {
        // Initializing hardware.
        leftParallelEncoder = hardwareMap.get(DcMotor.class, "left_y_encoder");
        rightParallelEncoder = hardwareMap.get(DcMotor.class, "right_y_encoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "x_encoder");

        // TODO: Reverse if necessary.
        //leftParallelEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightParallelEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        //perpendicularEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initializing localization system.

        // Initial position coordinates of the robot with a their distance unit.
        SmartPoint initialRobotPosition = new SmartPoint(
                Distance.DistanceUnit.CM,
                INITIAL_POS_X,
                INITIAL_POS_Y
                );

        // Initializing the measurement provider for the localization system.
        ThreeDeadWheelOdometry.MeasurementProvider measurementProvider = new ThreeDeadWheelOdometry.MeasurementProvider(
                perpendicularEncoder::getCurrentPosition,
                leftParallelEncoder::getCurrentPosition,
                rightParallelEncoder::getCurrentPosition,
                TICKS_TO_LINEAR_DISTANCE
        );

        // Initializing the localization system itself.
        odometry = new ThreeDeadWheelOdometry(initialRobotPosition, INITIAL_HEADING, ENCODER_WIDTH, measurementProvider);

        // Initializing localization system logging;\.
        logger = Debuggers.getGlobalLogger();
        odometry.attachLogger(logger);
    }

    @Override
    public void loop() {

        // Updating the calculations of the localization system.
        // This is NOT NECESSARY since it is usually being done automatically.
        odometry.update();

        // Getting the pose of the robot.
        // This is NOT NECESSARY since it is usually being done automatically.
        Pose2D pose = odometry.getPose();

        // Updating the debug information.
        odometry.debug();

        // Retrieving and displaying the debug information.
        DebugPacket[] loggerPackets = logger.dump();

        for (DebugPacket packet : loggerPackets) {
            telemetry.addData(packet.getHeader().getIdentifier(), packet.getValue());
        }
    }
}
