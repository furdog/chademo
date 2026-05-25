# LIN Bus protocol

This project is intended to implement hardware agnostic, P2P variation of LIN 2.1 protocol.

> [!CAUTION]
> This implementation does not follow LIN 2.1 specs strictly

# Implementation notes
> (14.05.2026)

So i plan to implement CSMA/CD like communication model on top of LIN
There are three steps involved:
- Carrier Sensing (Check if line is busy)
- Collision Detection (The device must listen itself and validate its output)
- Backoff Logic (If sent data does not match received data - STOP)

For now i have working TX automata. It only sends and does not listen to itself.
My current step is to implement carrier sensing logic.
The FSM must accept `IDLE` parameter `linbus_ack_idle(self)`

`linbus_queue_frame` Must fail if `IDLE` is set to `false`

In order to check if there's active carrier on the line, the hardware either can look for idle condition (interrupt),
or manually monitor recessive bit timeout. (Though this is not part of the implementation)

> (19.05.2026)

So i have finished initial LIN implementation, which now correctly transmits data.
I have also finished carrier detection and RX related logic.

Now i see that my automata is too big and there are too many similar, repetitive states and logic,
that can be simplified. But i'll keep going as it is for now.

Suggested change to refactor states:
```C
enum linbus_state {
	/** Not doing anything */
	LINBUS_STATE_IDLE,

	/** Wait for break condition (RX/TX) */
	LINBUS_STATE_LIN_BREAK,

	/** Wait for sync byte (RX/TX) */
	LINBUS_STATE_LIN_SYNC,

	/** Wait for PID (RX/TX) */
	LINBUS_STATE_LIN_PID,

	/** Wait for data bytes (len<=8) (RX/TX) */
	LINBUS_STATE_LIN_DATA,

	/** Wait for checksum (RX/TX) */
	LINBUS_STATE_LIN_CHECKSUM,

	/** Wait for idle condition (RX/TX) */
	LINBUS_STATE_LIN_IDLE,

	/** Frame has been sent */
	LINBUS_STATE_COMPLETE,

	/** Frame has been received with fault */
	LINBUS_STATE_FAULT
};
```
This will complicate automata and will require more careful RX/TX segragation.

> (20.05.2026)

So i faced more challenges with lin bus while testing it on real hardware.

First of all - interface is straight disaster. Idea is good, but it's almost unmanageable.
It requires major refactoring.

Debugging - disaster as well. It is spammy and can't be filtered. I had to perform some
adjustments while testing.

> (21.05.2026)

Before refactoring i'd like to make clear some problems:

- FSM step consist of partial steps.

Current design implies partial step automata, where every step does not switch to final state immediately.
After input parameters are changed. It does require multiple steps and every step may emit some events.
PROS: internal simplicity, HOOKS without callbacks. CONS: external api become hard to manage.

Switching to atomic steps, where one single step means all variety of outputs at the same time
will make the whole behaviour very solid.
PROS: Stable external behaviour. CONS: internal complexity.

Hybrid approach where:
External API kept atomic and internal API is partial unless explicitly told to be hooked.
PROS: Internal simplicity, Stable external behaviour. CONS: Too much effort to implement.

- Confirming events is devastating. There's certainly should be events confirmed selectively, via acknowledgement.

- Half-duplex. There's only one, either transmission or reception can work at the same time.
Though LIN is half-duplex protocol, there might be some deviations in the future that would
require full duplex link. Currently two instances of LIN automata may run simultaneously to achieve the goal.

- There's a high state overhead and state management is performed outside main FSM.
It makes API unstable as a whole. 

- Reading data should be symetrical to writing data and current implementation does not
make it clearly symetrical. TX and RX routines quite different.

- Debugging messages are too spammy and can't be selective.

- Carrier detection and idle detection logic may create racing conditions with main FSM,
as well as RX routine, where states are managed outside of main FSM.

- There's no way to tell whats current state is, neither there is no pre- and post- states.

- Events are emited only once and then never repeated, until previous event is acknowledged.
There is also other way - always emit an event even if nothing has changed internally.
Both kinds of emmiters are suitable for different purposes, but i suggest only one may exist to
reduce complexity.

> (22.05.2026)

I decided to simplify linbus.
My steps are:
- simplify _linbus_get_state_name
- do not switch states outside of FSM (WIP)
- deleting unnecessary states. (WIP)
- get rid of event acknowledgement.