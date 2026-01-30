
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

/**
 * Autonomous OpMode "AutoR12" for FTC robot using Pedro Pathing.
 * This class defines a state machine for following a sequence of paths.
 */
@Autonomous(name = "AutoR12", group = "Autonomous")
@Configurable // Enables Panels configuration if applicable
public class AutoR12 extends OpMode {
    private TelemetryManager panelsTelemetry; // Instance for managing telemetry panels
    public Follower follower; // Pedro Pathing follower for controlling robot movement
    private int pathState; // Current state in the autonomous state machine
    private Paths paths; // Container for pre-defined path chains

    private Timer pathTimer; // Timer to track time elapsed within each path state

    /**
     * Initializes the robot, follower, paths, and telemetry.
     */
    @Override
    public void init() {
        // Initialize Panels Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize the follower with hardware map and set starting pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Build all the paths defined in the inner Paths class
        paths = new Paths(follower);

        // Initialize path timer
        pathTimer = new Timer();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    /**
     * Main loop of the OpMode. Updates the follower and the state machine.
     */
    @Override
    public void loop() {
        follower.update(); // Update the Pedro Pathing follower to calculate powers
        autonomousPathUpdate(); // Update the autonomous sequence state machine

        // Log debugging information to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /**
     * Called when the "Play" button is pressed. Sets the initial path state.
     */
    @Override
    public void start(){
        setPathState(0);
    }

    /**
     * Inner class to define and build the path chains for the autonomous routine.
     */
    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

        /**
         * Constructor that builds all paths using the provided follower's path builder.
         * @param follower The follower instance used to build paths.
         */
        public Paths(Follower follower) {
            // Path 1: Initial movement
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 136.000),
                                    new Pose(54.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(129))
                    .build();

            // Path 2: Movement to second position
            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),
                                    new Pose(38.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180))
                    .build();

            // Path 3: Movement to third position
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(38.000, 84.000),
                                    new Pose(15.500, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 4: Return to a scoring/central position
            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.500, 84.000),
                                    new Pose(54.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))
                    .build();

            // Path 5: Move to pick up or interact at another point
            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),
                                    new Pose(40.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180))
                    .build();

            // Path 6: Straight line movement
            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 60.000),
                                    new Pose(15.500, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            // Path 7: Curved movement back
            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.500, 60.000),
                                    new Pose(39.745, 59.547),
                                    new Pose(54.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))
                    .build();

            // Path 8: Next movement in sequence
            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 85.000),
                                    new Pose(40.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180))
                    .build();

            // Path 9: Straight line with tangent heading
            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.000, 36.000),
                                    new Pose(14.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            // Path 10: Final movement in the defined sequence
            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.000, 36.000),
                                    new Pose(54.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))
                    .build();
        }
    }

    /**
     * Manages the sequence of paths to follow. 
     * Each state corresponds to following a specific PathChain.
     */
    public void autonomousPathUpdate() {
        switch(pathState){
            case 0:
                // Start following the first path
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                // Wait until Path1 is finished, then start Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait until Path2 is finished, then start Path3
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;

            case 3:
                // Wait until Path3 is finished, then start Path4
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait until Path4 is finished, then start Path5
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;

            case 5:
                // Wait until Path5 is finished, then start Path6
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(6);
                }
                break;

            case 6:
                // Wait until Path6 is finished, then start Path7
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    setPathState(7);
                }
                break;

            case 7:
                // Wait until Path7 is finished, then start Path8
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8);
                    setPathState(8);
                }
                break;

            case 8:
                // Wait until Path8 is finished, then start Path9
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    setPathState(9);
                }
                break;

            case 9:
                // Wait until Path9 is finished, then start Path10
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10);
                    setPathState(10);
                }
                break;

            case 10:
                // Wait until Path10 is finished, then end the sequence
                if (!follower.isBusy()) {
                    setPathState(-1); // End of autonomous
                }
                break;
        }
    }

    /**
     * Updates the current path state and resets the path timer.
     * @param pState The new state to transition to.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
