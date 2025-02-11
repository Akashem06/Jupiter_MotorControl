
## Sensorless Control loop

## Hall sensor Control loop

## Current Controlled V.S Velocity Controlled

Current controlled refers to an electric motor where the speed and torque are regulated by controlled the electrical current flowing through it.
Since torque is directly related to current, controlled the current allows for percise torque control.
Take the torque constant (Kt) where Torque = Kt * Current, clearly current is directly porportional to torque.
Current control is better for applications using constant torque, or when torque is more important than the speed output. The way it works in application
is the motor controller reads the phase current of the floating phase, and based on the setpoint, modifies the motor voltage via the Inverter PWM.

Vvelocity control refers to an electric motor where the current and torque are regulated by measuring the speed. It automatically adjusts
the PWM duty cycle, which indirectly increases current flow, creating more torque and causing acceleration to maintain the speed.

An easy way to think of current control, is like maintaining constnt pressure on a gas pedal, whereas velocity control is like
cruise control, which automatically maintains speed.

You'll notice that the motor controller is managed through the Inverter PWM, which controls the voltage seen by the motor. This will indirectly
control the output current via Ohm's law.
