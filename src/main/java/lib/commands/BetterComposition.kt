package lib.commands

import edu.wpi.first.wpilibj2.command.Command


infix fun Command.then(other: Command): Command {
    return this.andThen(other)
}

infix fun Command.with(other: Command): Command {
    return this.alongWith(other)
}

infix fun Command.race(other: Command): Command {
    return this.raceWith(other)
}

infix fun Command.deadline(other: Command): Command {
    return this.deadlineWith(other)
}