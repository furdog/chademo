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

TODO rename `linbus_queue_frame` to `linbus_send_frame`

In order to check if there's active carrier on the line, the hardware either can look for idle condition (interrupt),
or manually monitor recessive bit timeout. (Though this is not part of the implementation)