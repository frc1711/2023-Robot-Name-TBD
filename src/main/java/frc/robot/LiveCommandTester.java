package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import claw.logs.CLAWLogger;
import claw.rct.commands.CommandProcessor;
import claw.rct.commands.CommandReader;
import claw.rct.commands.CommandProcessor.BadCallException;
import claw.rct.network.low.ConsoleManager;
import claw.rct.network.low.Waiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class LiveCommandTester <T> {
    
    private final Supplier<T> inputSupplier;
    private final Consumer<T> periodicExecute;
    private final Runnable putInSafeState;
    private final Subsystem[] subsystems;
    
    public LiveCommandTester (Supplier<T> inputSupplier, Consumer<T> periodicExecute, Runnable putInSafeState, Subsystem... subsystems) {
        this.inputSupplier = inputSupplier;
        this.periodicExecute = periodicExecute;
        this.putInSafeState = putInSafeState;
        this.subsystems = subsystems;
    }
    
    public CommandProcessor toCommandProcessor (String commandName) {
        return new CommandProcessor(
            commandName,
            commandName,
            "Use this command to run a custom test command on the robot.",
            this::runCommand
        );
    }
    
    public class TestCommand extends CommandBase {
        public TestCommand () {
            addRequirements(subsystems);
        }
        
        @Override
        public void initialize () {
            putInSafeState.run();
        }
        
        @Override
        public void execute () {
            periodicExecute.accept(inputSupplier.get());
        }
        
        @Override
        public void end (boolean interrupted) {
            putInSafeState.run();
        }
    }
    
    private void runCommand (ConsoleManager console, CommandReader reader) throws BadCallException {
        reader.allowNone();
        console.println("Press enter to start the command test. Double-tap enter at any time to disable the robot.");
        
        // Wait until input after the driverstation is enabled
        console.readInputLine();
        while (DriverStation.isDisabled()) {
            console.printlnErr("Enable the robot and try again.");
            console.readInputLine();
        }
        
        // Run the command
        TestCommand command = new TestCommand();
        command.withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
        console.println("Running");
        
        while (!console.hasInputReady()) {
            // Do nothing here, just wait until input is ready
        }
        
        // Stop the command
        console.println("Stopping command");
        command.cancel();
        
    }
    
}
