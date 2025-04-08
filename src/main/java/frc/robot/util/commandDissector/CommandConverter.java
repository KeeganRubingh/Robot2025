package frc.robot.util.commandDissector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.TreeMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandConverter {
    public static class CommandBranch {
        public final String name;
        public final String description;
        public final Color branchColor;
        public final Command me;
        public final SupportedCommandType type;
        public final List<CommandBranch> subBranches;
        public final List<String> branchLabels;

        public CommandBranch(Color branchColor, Command me, List<CommandBranch> subBranches) {
            this.name = me.getName();
            this.description = getReadableRequirements(me);
            this.branchColor = branchColor;
            this.me = me;
            this.type = SupportedCommandType.getSupportedTypeOf(me);
            this.subBranches = subBranches;
            this.branchLabels = new ArrayList<>();
        }

        public CommandBranch(Color branchColor, Command me, List<CommandBranch> subBranches, List<String> labels) {
            this.name = me.getName();
            this.description = getReadableRequirements(me);
            this.branchColor = branchColor;
            this.me = me;
            this.type = SupportedCommandType.getSupportedTypeOf(me);
            this.subBranches = subBranches;
            this.branchLabels = labels;
        }

        public void addChild(CommandBranch... children) {
            subBranches.addAll(Arrays.asList(children));
        }

        @Override
        public String toString() {
            return "{%nname:%s,%ndescription:%s,%ncolor:%s,%ncommand:%s,%ntype:%s,%nchildren:%s%n,labels:%s%n}".formatted(
                name,description,branchColor,me,type,subBranches.toString(),branchLabels.toString()
            );
        }
    }

    public static enum SupportedCommandType{
        LEAF("NONE_DONOTSET") {
            public CommandBranch makeInstance(Object obj) {
                Command leaf = (Command) obj;
                return new CommandBranch(Color.kGreen, leaf, new ArrayList<>());
            }
        },
        CONDITIONAL("ConditionalCommand"){
            public CommandBranch makeInstance(Object obj) {
                ConditionalCommand group = (ConditionalCommand) obj;
                List<CommandBranch> subcmds = new ArrayList<>();
                List<String> labels = new ArrayList<>();

                try {
                    group.getClass().getDeclaredField("m_onTrue").setAccessible(true);
                    group.getClass().getDeclaredField("m_onFalse").setAccessible(true);
                    Command onTrue = (Command) ConditionalCommand.class.getDeclaredField("m_onTrue").get(group);
                    Command onFalse = (Command) ConditionalCommand.class.getDeclaredField("m_onFalse").get(group);
                    subcmds.add(convert(onTrue));
                    labels.add("True");
                    subcmds.add(convert(onFalse));
                    labels.add("False");
                } catch (Exception e) {
                    System.err.println(e);
                    return null;
                }

                return new CommandBranch(
                    Color.kAliceBlue, 
                    group, 
                    subcmds);
            }
        },
        SEQUENTIAL("SequentialCommandGroup"){
            public CommandBranch makeInstance(Object obj) {
                SequentialCommandGroup sequentialGroup = (SequentialCommandGroup) obj;

                SequentialCommandGroup group =  sequentialGroup;
                List<Command> extracted;
                List<CommandBranch> sequence = new ArrayList<>();

                try {
                    SequentialCommandGroup.class.getField("m_commands").trySetAccessible();
                    @SuppressWarnings(value = "unchecked") 
                    List<Command> testL = (List<Command>) SequentialCommandGroup.class.getField("m_commands").get(group);
                    extracted = testL;
                } catch (Exception e) {
                    System.err.println(e);
                    return null;
                }
                for(int i = 0; i < extracted.size(); i++) {
                    CommandBranch command = convert(extracted.get(0));
                    if(command.type == this) {
                        sequence.addAll(command.subBranches);
                    } else {
                        sequence.add(command);
                    }
                }
                return new CommandBranch(Color.kAliceBlue, sequentialGroup, sequence);
            }
        },
        PARALLEL("ParallelCommandGroup"){
            public CommandBranch makeInstance(Object obj) {
                ParallelCommandGroup group = (ParallelCommandGroup) obj;

                List<Command> extracted;
                List<CommandBranch> sequence = new ArrayList<>();

                try {
                    @SuppressWarnings(value = "unchecked") 
                    List<Command> testL = (List<Command>) SequentialCommandGroup.class.getField("m_commands").get(group);
                    extracted = testL;
                } catch (Exception e) {
                    System.err.println(e);
                    return null;
                }
                for(int i = 0; i < extracted.size(); i++) {
                    CommandBranch command = convert(extracted.get(0));
                    if(command.type == this) {
                        sequence.addAll(command.subBranches);
                    } else {
                        sequence.add(command);
                    }
                }

                return new CommandBranch(Color.kAliceBlue, group, sequence);
            }
        };

        public final String className;
        public abstract CommandBranch makeInstance(Object command);

        private SupportedCommandType(String name) {
            this.className = name;
        }

        public static SupportedCommandType getSupportedTypeOf(Command c) {
            String typename = c.getClass().getSimpleName();
            System.out.println(typename);
            for(SupportedCommandType type : SupportedCommandType.values()) {
                System.out.println(type.className);
                if(type.className.equals(typename)) {
                    return type;
                }
            }
            return SupportedCommandType.LEAF;
        }

        public static CommandBranch convertCommand(Command cmd) {
            return getSupportedTypeOf(cmd).makeInstance(cmd);
        }
    }

    public static String getReadableRequirements(Command cmd) {
        List<String> readableNames = new ArrayList<>();

        for(Subsystem s : cmd.getRequirements()) {
            readableNames.add(s.getClass().getSimpleName());
        }

        return readableNames.toString();
    }

    public static CommandBranch convert(Command toConvert) {
        System.out.println("STARTING CONVERSION LOOK AT ME!!!");
        return SupportedCommandType.convertCommand(toConvert);
    }
}
