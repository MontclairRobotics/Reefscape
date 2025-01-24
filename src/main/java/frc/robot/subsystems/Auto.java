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
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.ScoringLevel;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.PoseUtils;

public class Auto extends SubsystemBase {

    private String prevAutoString = "";
    private ArrayList<PathPlannerPath> pathList = new ArrayList<PathPlannerPath>();

    private Command autoCmd = Commands.none();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable auto = inst.getTable("Auto");

    StringTopic autoTopic = auto.getStringTopic("Auto String");
    StringEntry stringEnt = autoTopic.getEntry("");

    StringTopic feedbackTopic = auto.getStringTopic("Auto Feedback");
    StringPublisher feedbackPub = feedbackTopic.publish();

    Alliance prevAlliance = Alliance.Blue;

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

    public boolean isAutoStringValid(String autoString) {

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
                setFeedback("Character " + (i + 1) + ": " + autoString.charAt(i) + " is not a scoring location, when it should be!",
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
                setFeedback("Character " + (i + 3) + ": " + autoString.charAt(i + 2) + " is not a valid pickup location, when it should be!",
                        NotificationLevel.ERROR);
                return false;
            }

        }

        setFeedback("Auto String Valid!", NotificationLevel.INFO);
        return true;

    }

    public void drawPaths() { // TODO rotate for red alliance?
        for (int i = 0; i < pathList.size(); i++) {
            PathPlannerPath path = pathList.get(i);

            Pose2d[] posList = path.getPathPoses().toArray(new Pose2d[0]);
            for (int j = 0; j < posList.length; j++) {
                posList[j] = PoseUtils.flipPoseAlliance(posList[j]);
            }
            // Trajectory traj = TrajectoryGenerator.generateTrajectory(path.getPathPoses(), new TrajectoryConfig(Drivetrain.MAX_SPEED, Drivetrain.FORWARD_ACCEL));
            field.getObject("obj" + i).setPoses(posList);
            // field.getObject("obj" + i).setTrajectory(traj);

        }
    }

    public Command buildAutoCommand(String autoString) {
        if (!isAutoStringValid(autoString)) {
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
            String middleChar = "-";
            boolean firstPath = false;
            if (first != null && second != null) {
                if (Character.isLowerCase(first.charAt(0)) || Character.isLowerCase(second.charAt(0))) {
                    middleChar = "_";
                }
                pathName = first + middleChar + second;
                if (i == 1) {
                    pathName = "S" + pathName;
                    firstPath = true;
                }
                pathName = i == 1 ? "S" + first + middleChar + second : first + middleChar + second; // makes sure it accounts starting
                                                                                    // path having an "S"

                try {
                    // Load the path you want to follow using its name in the GUI

                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

                    // Store path to be drawn on dashboard
                    // Create a path following command using AutoBuilder. This will also trigger
                    // event markers.
                    path1Cmd = AutoBuilder.followPath(path);

                    if (firstPath) {
                        Optional<Pose2d> opPose = path.getStartingHolonomicPose();
                        Pose2d pose = opPose.isPresent() ? PoseUtils.flipPoseAlliance(opPose.get()) : new Pose2d();
                        autoCommand.addCommands(Commands.runOnce(() -> {
                            RobotContainer.drivetrain.swerveDrive.resetPose(pose);
                        }));

                        //TODO this can be deleted, is here for testing purposes
                        RobotContainer.drivetrain.swerveDrive.resetPose(pose);
                    }


                    // TODO needs to be .generateTrajectory()? maybe only if the ideal one doesn't
                    // exist?
                    pathList.add(path);
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    setFeedback("Path " + pathName + " not found!", NotificationLevel.ERROR);
                    pathList.clear();
                    return Commands.none();
                }
            }

            if (second != null && fourth != null) {
                    middleChar = "-";
                if (Character.isLowerCase(third.charAt(0)) || Character.isLowerCase(fourth.charAt(0))) {
                    middleChar = "_";
                }
                pathName = second + middleChar + fourth;
                try {
                    // Load the 2nd path you want to follow using its name in the GUI

                    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

                    // Store path to be drawn on dashboard
                    // Create a path following command using AutoBuilder. This will also trigger
                    // event markers.
                    path2Cmd = AutoBuilder.followPath(path);

                    // TODO needs to be .generateTrajectory()? maybe only if the ideal one doesn't
                    // exist?
                    pathList.add(path);
                } catch (Exception e) {
                    DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                    setFeedback("Path " + pathName + " not found!", NotificationLevel.ERROR);
                    return Commands.none();
                }
            }

            
            if (third != null) {
                autoCommand.addCommands(Commands.none()); //Elevator target height command
            }
            ScoringLevel.L4.getHeight();

            


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

    public void validateAndCreatePaths(String str) {
        clearField();
        if (isAutoStringValid(str)) {
            autoCmd = buildAutoCommand(str);
            drawPaths();
        }
    }

    public void clearField() {
        pathList.clear();
        for (int i  = 0; i < 100; i++) {
            FieldObject2d obj = field.getObject("obj" + i);
            obj.setTrajectory(new Trajectory());
        }
    }

    public Command getAutoCommand() {
        return autoCmd;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            String str = stringEnt.get("");
            SmartDashboard.putData(field);

            // String autoString = str.replace(' ', Character.MIN_VALUE); // check
            String autoString = "";
            Optional<Alliance> alliance = DriverStation.getAlliance();
            for (char a : str.toCharArray()) {
                if (a != ' ' && a != ',') {
                    autoString += a;
                }
            }
            if (!autoString.equals(prevAutoString)) {
                prevAutoString = autoString;
                validateAndCreatePaths(autoString);
            }
            else if (alliance.isPresent() && alliance.get() != prevAlliance) {
                prevAlliance = alliance.get();
                validateAndCreatePaths(autoString);
            }
        }

        if (DriverStation.isAutonomous() || DriverStation.isDisabled()) {
            field.setRobotPose(RobotContainer.drivetrain.getRobotPose());
        }
    }
}
