package swervelib.simulation.ironmaple.simulation.opponents;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.simulation.ironmaple.simulation.opponentsim.OpponentManager;
import swervelib.simulation.ironmaple.simulation.opponentsim.SmartOpponent;
import swervelib.simulation.ironmaple.simulation.opponentsim.SmartOpponentConfig;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class EmptyOpponent extends SmartOpponent {
    private Optional<Supplier<Pose2d>> defenseTarget = Optional.empty();
    /**
     * A Dumb SmartOpponent.
     * Can only do default SmartOpponent things. Non Season Specific
     * Loads with default initial starting and queening poses. Set manually for more than one bot.
     * Check {@link OpponentManager} for some poses.
     *
     * @param name     the opponent name. Typically just "Defense Bot 1".
     *                 Names should not be the same.
     * @param alliance the opponents {@link DriverStation.Alliance}.
     */
    public EmptyOpponent(String name, DriverStation.Alliance alliance) {
        /// All Options should be set in the constructor.
        super(new SmartOpponentConfig()
                .withName(name)
                .withAlliance(alliance)
                .withQueeningPose(new Pose2d(-6, 0, new Rotation2d()))
                .withStartingPose(new Pose2d(15, 6, Rotation2d.fromDegrees(180)))
                .withChassisConfig(SmartOpponentConfig.ChassisConfig.Presets.SimpleSquareChassis.getConfig()
                        .withMaxLinearVelocity(MetersPerSecond.of(6))
                        .withMaxAngularVelocity(DegreesPerSecond.of(360)))
                .withAutoEnable());
    }

    /**
     * The collect state to run.
     * Does nothing
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command collectState() {
        return Commands.none();
    }

    /**
     * The score state to run.
     * Does nothing
     *
     * @return a runnable that runs the state.
     */
    @Override
    protected Command scoreState() {
        return Commands.none();
    }

    // TODO
    public EmptyOpponent withXboxController(CommandXboxController xboxController) {
        config.withJoystick(xboxController);
        config.withState("Joystick", this::joystickState);
        config.withBehavior(
                "Player",
                startingState("Joystick").andThen(startingState("Joystick").ignoringDisable(false)));
        config.updateBehaviorChooser();
        /// Enable Manipulator control
        xboxController.leftBumper().and(config.isStateTrigger("Joystick")).whileTrue(manipulatorSim.intake("Intake"));
        xboxController.rightBumper().and(config.isStateTrigger("Joystick")).whileTrue(manipulatorSim.score("Coral"));
        return this;
    }

    /**
     * Defense State to simply follow the defense target if present.
     * Pretty demanding since it just refreshes a new pathfind constantly.
     *
     * @return a {@link Command} to run defense.
     */
    private Command defenseState() {
        return Commands.defer(() ->
                pathfind(Pair.of("Defense Target", defenseTarget.orElse(this::getOpponentPose).get()), config.chassis.maxLinearVelocity),
                Set.of(this)).withTimeout(.5).repeatedly();
    }

    /**
     * Enables a dumb defense bot to be an annoyance.
     *
     * @param defenseTarget the pose to attack.
     * @return this, for chaining or something.
     */
    public EmptyOpponent withDefense(Supplier<Pose2d> defenseTarget) {
        this.defenseTarget = Optional.of(defenseTarget);
        config.withState("Defense", this::defenseState);
        config.withBehavior(
                "Defense",
                startingState("Defense").ignoringDisable(false));
        config.updateBehaviorChooser();
        return this;
    }

    /**
     * The joystick state to run. Should be inaccessible when not set.
     *
     * @return the joystick state to run.
     */
    private Command joystickState() {
        final CommandXboxController xbox = ((CommandXboxController) config.getJoystick());
        return drive(
                () -> new ChassisSpeeds(
                        MathUtil.applyDeadband(
                                xbox.getLeftY() * -config.chassis.maxLinearVelocity.in(MetersPerSecond),
                                config.joystickDeadband),
                        MathUtil.applyDeadband(
                                xbox.getLeftX() * -config.chassis.maxLinearVelocity.in(MetersPerSecond),
                                config.joystickDeadband),
                        MathUtil.applyDeadband(
                                xbox.getRightX() * -config.chassis.maxAngularVelocity.in(RadiansPerSecond),
                                config.joystickDeadband)),
                false);
    }
}
