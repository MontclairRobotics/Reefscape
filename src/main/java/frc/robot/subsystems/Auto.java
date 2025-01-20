package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.CoralScoringLevel;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class Auto extends SubsystemBase {

    private String autoString = "";
    private String prevAutoString = "";
    private ArrayList<PathPlannerTrajectory> pathList = new ArrayList<PathPlannerTrajectory>();

    private Command autoCmd = Commands.none();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable auto = inst.getTable("Auto");

    StringTopic autoTopic = auto.getStringTopic("Auto String");
    StringEntry stringEnt = autoTopic.getEntry("");

    StringTopic feedbackTopic = auto.getStringTopic("Auto Feedback");
    StringPublisher feedbackPub = feedbackTopic.publish();

    public static Field2d field = new Field2d();

    public void setFeedback(String s, NotificationLevel level) {
        feedbackPub.set(s);
        Notification not = new Notification(level, "Auto Feedback", s, 3000);
        Elastic.sendNotification(not);
    }

    public Auto() {
        autoTopic.setRetained(true); // Should be retained?
        stringEnt.set("");
        feedbackTopic.setRetained(true);
        feedbackPub.set("Enter auto string!");
    }

    /* ArrayLists to hold the values of various waypoints */
    private ArrayList<String> startingLocations = new ArrayList<String>(
            Arrays.asList("S1", "S2", "S3", "S4", "S5"));
    private ArrayList<String> scoringLocations = new ArrayList<String>(
            Arrays.asList("A", "a", "B", "b", "C", "c", "D", "d", "E", "e", "F", "f"));
    private ArrayList<String> pickupLocations = new ArrayList<String>(
            Arrays.asList("1", "2", "3", "4", "5", "6"));
    private ArrayList<String> coralLevels = new ArrayList<String>(
            Arrays.asList("1", "2", "3", "4"));

    public boolean isAutoStringValid() {

        if (autoString.length() < 2) {
            setFeedback("Not long enough!", NotificationLevel.ERROR);
            return false;
        }

        /* Checks if there is a valid starting location */
        if (!startingLocations.contains(autoString.substring(0, 2))) {
            setFeedback("Invalid starting location", NotificationLevel.ERROR);
            return false;
        }

        /* Starts at 2 because you don't need to loop through the starting location */
        for (int i = 2; i < autoString.length(); i += 3) { // Loops every 3

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i, i + 1);
            String second = i < autoString.length() - 1 ? autoString.substring(i + 1, i + 2) : null;
            String third = i < autoString.length() - 2 ? autoString.substring(i + 2, i + 3) : null;

            /* Checks if we are going to a valid scoring location */
            if (!scoringLocations.contains(first)) {
                setFeedback("Character " + (i + 1) + " is not a scoring location, when it should be!",
                        NotificationLevel.ERROR);
                return false;
            }
            /* Checks if the coral level we want to score at is valid */
            if (second == null || !coralLevels.contains(second)) {
                setFeedback("Invalid or missing Coral level at " + (i + 2),
                        NotificationLevel.ERROR);
                return false;
            }
            /* Checks if the pickup location we want to score at is valid */
            if (third != null && !pickupLocations.contains(third)) {
                setFeedback("Character " + (i + 3) + " is not a valid pickup location, when it should be!",
                        NotificationLevel.ERROR);
                return false;
            }

        }

        setFeedback("Auto String Valid!", NotificationLevel.INFO);
        return true;

    }

    public void drawPaths() { // TODO rotate for red alliance?
        for (int i = 0; i < pathList.size(); i++) {
            System.out.println("PathList size: " + pathList.size());
            System.out.println("Loop Index: " + i);
            PathPlannerTrajectory ppTraj = pathList.get(i);

            ArrayList<Pose2d> poseList = new ArrayList<Pose2d>(pathList.size());

            for (PathPlannerTrajectoryState state : ppTraj.getStates()) {
                poseList.add(state.pose);
                System.out.println(state.pose);
            }

            // TODO might need to do this using states to account for different constraints
            // at different points.
            // I'm not sure if it matters since theoretically Pathplanner has already done
            // the math?
            try {
            System.out.println("Length: " + poseList.size());
                Trajectory traj = TrajectoryGenerator.generateTrajectory(poseList,
                        new TrajectoryConfig(Drivetrain.MAX_SPEED, Drivetrain.FORWARD_ACCEL));
                field.getObject("obj" + i).setTrajectory(traj);
            } catch (Exception e) {
                setFeedback("Had to skip drawing trajectory", NotificationLevel.WARNING);
            }

        }
    }

    public Command buildAutoCommand() {
        if (!isAutoStringValid()) {
            return Commands.none();
            // Possibly make this return just a simple path so we get the leave bonus
        }

        SequentialCommandGroup autoCommand = new SequentialCommandGroup();

        autoCommand.addCommands(Commands.none()); // TODO Pose init command

        // S1 B3 1
        // TODO: write the rest of this
        for (int i = 1; i < autoString.length(); i += 3) {

            /* Creates strings to store each part of the 3 character interval */
            String first = autoString.substring(i, i + 1); // pickup location
            String second = i < autoString.length() - 1 ? autoString.substring(i + 1, i + 2) : null; // scoring location
            String third = i < autoString.length() - 2 ? autoString.substring(i + 2, i + 3) : null; // coral level
            String fourth = i < autoString.length() - 3 ? autoString.substring(i + 3, i + 4) : null; // next pickup
                                                                                                     // location

            Command path1Cmd = Commands.none();
            Command path2Cmd = Commands.none();
            //S1 B 1 1 A 1

            /* adds command to from pickup location to scoring location */
            String pathName;
            if (first != null && second != null) {
                pathName = i == 1 ? "S" + first + "-" + second : first + "-" + second; // makes sure it accounts starting
                                                                                    // path having an "S"

                try {
                    // Load the path you want to follow using its name in the GUI

                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                    System.out.println("Path1: " + pathName);

                    // Store path to be drawn on dashboard
                    // Create a path following command using AutoBuilder. This will also trigger
                    // event markers.
                    path1Cmd = AutoBuilder.followPath(path);


                    // TODO needs to be .generateTrajectory()? maybe only if the ideal one doesn't
                    // exist?
                    Optional<PathPlannerTrajectory> ppTraj = path.getIdealTrajectory(RobotConfig.fromGUISettings());
                    if (ppTraj.isPresent()) {
                        pathList.add(ppTraj.get());
                    } else {
                        setFeedback("Trajectory for " + pathName + " not present!", NotificationLevel.ERROR);
                    }
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    setFeedback("Path " + pathName + " not found!", NotificationLevel.ERROR);
                    return Commands.none();
                }
            }

            pathName = third != null && fourth != null ? second + "-" + fourth : null;

            if (pathName != null) {
                try {
                    // Load the path you want to follow using its name in the GUI

                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                    System.out.println("Path2: " + pathName);

                    // Store path to be drawn on dashboard
                    // Create a path following command using AutoBuilder. This will also trigger
                    // event markers.
                    path2Cmd = AutoBuilder.followPath(path);

                    // TODO needs to be .generateTrajectory()? maybe only if the ideal one doesn't
                    // exist?
                    Optional<PathPlannerTrajectory> ppTraj = path.getIdealTrajectory(RobotConfig.fromGUISettings());
                    if (ppTraj.isPresent()) {
                        pathList.add(ppTraj.get());
                    } else {
                        System.out.println("TRAJECTORY NOT PRESENT _____________________________-");
                    }
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    setFeedback("Path " + pathName + " not found!", NotificationLevel.ERROR);
                    return Commands.none();
                }
            }

            


            //TODO we may need some way to identify whether we are going to a pickup zone or a scoring zone.
            //right now there is no disinction

            /* ADDS SCORING COMMAND */
            autoCommand.addCommands(Commands.none()); // change this to score whatever level you want to score

            /* adds command form scoring location to next pickup location */
            autoCommand.addCommands(path1Cmd);

            /* ADDS AN INTAKING COMMAND */
            autoCommand.addCommands(Commands.none()); // change this to intaking command

            autoCommand.addCommands(path2Cmd); //TODO work out when these should be added?

        }

        setFeedback("Auto built successfully!", NotificationLevel.INFO);
        return autoCommand;
    }

    public void validateAndCreatePaths() {
        clearField();
        if (isAutoStringValid()) {
            autoCmd = buildAutoCommand();
            drawPaths();
        }
    }

    public void clearField() {
        pathList.clear();
        for (int i  = 0; i < 100; i++) {
            FieldObject2d obj = field.getObject("obj" + i);
            // obj.setPose(new Pose2d(-100, -100, Rotation2d.fromDegrees(0)));
            obj.setTrajectory(new Trajectory());
        }
        // field.close();
        // field = new Field2d();
    }

    public Command getAutoCommand() {
        return autoCmd;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            String str = stringEnt.get("");
            SmartDashboard.putData(field);

            autoString = str.replace(' ', Character.MIN_VALUE); // check
            if (!autoString.equals(prevAutoString)) {
                System.out.println(autoString + "-------------------");
                prevAutoString = autoString;
                validateAndCreatePaths();
            }
        }
    }
}
