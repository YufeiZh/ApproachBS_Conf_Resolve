# Reward Function Designing

## Principles

### Encourage:
**E1.** Vectoring the arrival aircrafts to the final approach points (FAPs) of their assigned runways, then direct them to land on the runways
**E2.** Vectoring or direct the departrue aircrafts to the assigned departure waypoints

### Discourage:

*Extent(colors): <font color=red>severe(red)</font>, <font color=magenta>heavy(magenta)</font>, <font color=orange>medium(orange)</font>, <font color=green>light(green)</font>*

<font color=red>**D1.** Aircraft conflicts</font>
<font color=red>**D2.** Restricted area invasions</font>
<font color=red>**D3.** Leaving airspace unexpectly (flying too low, too high, or out of range)</font>
<font color=magenta>**D4.** Aircrafts staying too long in the airspace</font>
<font color=orange>**D5.** Over-flying airports in low altitude, unless the aircraft is departing from or approaching the designated airport</font>
<font color=orange>**D6.** High airspeed in low altitude or low airspeed in high altitude</font>
<font color=magenta>**D7.** For arrival aircrafts, making them direct to destination (by sending direct-to commands) too early, or when the headings/altitudes/airspeeds are inappropriate</font>
<font color=orange>**D8.** Sending commands to a single aircraft too frequently or too infrequently</font>
<font color=orange>**D9.** Interrupting aircrafts' self-navigation (after they received a "direct-to" command) by sending heading, altitude, or speed change commands</font>
<font color=orange>**D10.** Making aircrafts do sharp turns by sending commands</font>
<font color=green>**D11.** Making arrival aircrafts climb (increase altitude), or making departure aircrafts descend (decrease altitude) by sending commands</font>
<font color=green>**D12.** Making arrival aircrafts accelerate (increase airspeed), or making departure aircrafts decelerate (decrease airspeed) by sending commands</font>