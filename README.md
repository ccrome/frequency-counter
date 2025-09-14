# Frequency Counter!

A simple frequency counter, using a GPS PPS signal as the reference.

Basically, it clocks GPT2 with the signal of interest on pin 14, and uses an input capture on the timer as the time base.

So, it counts the number of counts between PPS signals

| Pin | Function |
|-----|----------|
| 14  | 10MHz signal in |
| 15  | GPS PPS signal in |
