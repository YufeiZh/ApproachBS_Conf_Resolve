# Action Space of Autonomous TRACON

This is a guideline to the action space of the Atonomous TRACON, which is realized with BlueSky Simulator together with the approachbs plugin.

Each action is an ATC command sent to an aircraft using the *stack commands* of BlueSky. Instructions of changing heading, altitude, and speed can be sent simultenuously. For example, to an aircraft with callsign `DAL99`, the instructions  
```
HDG DAL99, 350; SPD DAL99, 210
```
should be considered as one command (action).

Assume from now on that:
* We import `bluesky` as `bs` with the plugin `approachbs.py`.
* We have `approach_bs = bs.plugins.approachbs.approach_bs`. This is how we access the additional attributes of the aircrafts.
* For an aircraft with callsign `acid` (string), use `bs.traf.id2idx(acid)` to find its index.
* A stack command `COMMAND` (string) can be sent via `bluesky.stack.stack(COMMAND)`.  

## Actions:

The action space should be *continuous*, but rounded when excuted.

For an aircraft with callsign `acid` (string) and index `idx = bs.traf.id2idx(acid)`, a command sent to it may consist of the following instruction(s):

* **Send command**: binary. Whether to send a command or not.

* **Direct to**: binary. Whether to direct an aircraft to its target waypoint (or landing runway) or not. The intention of an aircraft can be obtained from `approach_bs.intention` and the assigned target waypoint (or landing runway) can be obtained from `approach_bs.wp_id`.
    * If `approach_bs.intention[idx]` is `0`, it is an arrival aircraft. The assigned runway `approach_bs.wp_id[idx]` is of the form "{Airport ID}/{Runway ID}", e.g., `"PHNL/04L"`. To direct the aircraft to land on the assigned runway, use stack command
    ```
    DEST {acid}, PHNL, RW04L
    ```
    (Remember to add "RW" before the runway id otherwise it won't be parsed by BlueSky.)  
    * If `approach_bs.intention[idx]` is `1`, it is a departure aircraft. The assigned departure waypoint `approach_bs.wp_id[idx]` is the identifier of the departure waypoint. To direct the aircraft to the waypoint, use stack command
    ```
    ADDWPT {acid}, {approach_bs.wp_id[idx]}
    ``` 

*The following instructions can be sent only if we decide to send a command which is not "direct to". In other words, if a "direct to" command is included, then the following instructions should not be included.*

* **Change heading**: round to integers (degree) in $[0,359]$. To assign a heading `hdg`, use stack command
    ```
    HDG {acid}, {hdg}
    ```

* **Change altitude**: round to $100$ feet, range from `approach_bs.tracon.bottom` to `approach_bs.tracon.top`. To assign an altitude `alt`, use stack command
    ```
    ALT {acid}, {alt}
    ```

* **Change speed**: round to integers (knots) in $[160, 280]$. To assign a speed `spd`, use stack command
    ```
    SPD {acid}, {spd}
    ```




