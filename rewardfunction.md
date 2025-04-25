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
<font color=magenta>**D5.** For arrival aircrafts, making them direct to destination (by sending direct-to commands) too early, or when the headings/altitudes/airspeeds are inappropriate</font>  
<font color=orange>**D6.** Sending commands to a single aircraft too frequently or too infrequently or sending commands to an aircraft that is not in-airspace (not under control)</font>  
<font color=orange>**D7.** Interrupting aircrafts' self-navigation (after they received a "direct-to" command) by sending heading, altitude, or speed change commands</font>  
<font color=orange>**D8.** Making aircrafts do sharp turns by sending commands</font>  
<font color=green>**D9.** Making arrival aircrafts climb (increase altitude), or making departure aircrafts descend (decrease altitude) by sending commands</font>  
<font color=green>**D10.** Making arrival aircrafts accelerate (increase airspeed), or making departure aircrafts decelerate (decrease airspeed) by sending commands</font>  

## Details

Some predefined variables and conventions used in the following instructions:  
1. Input arguments:  
    * `traf` is the instance of the traffic of BlueSky. It contains all the aircrafts' attributes.  
    * `simt` is the current simulation time in seconds (float).  
    * `approach_bs` is the instance of the approachbs plugin. It contains all the aircrafts' additional attributes and the airspace (including airports) info.  
    * `monitor` is the instance of a class monitored aircrafts' previous status. It should be implemented in the module which calls this function.  
    * `actions` is the most recent actions. It is a dictionary with keys as the aircraft callsigns and values as the actions. (The format of the actions is not designed yet and the actions are described in `actionspace.md`.)  
    * `max_in_time` is the expected maximum time (in seconds) that an arrival aircraft can stay in the airspace. It will change dinamically during the training process.  
    * `max_out_time` is the expected maximum time (in seconds) that a departure aircraft can stay in the airspace. This will change dinamically during the training process.    
    * `max_actions_per_minute` is the maximum number of actions that can be sent to an aircraft in a minute.    
    * `min_actions_per_minute` is the minimum number of actions that can be sent to an aircraft in a minute.    

2. The attributes of aircrafts are numpy arrays of length `traf.ntraf`. In the instructions below, for each aircraft with callsign `acid` (string) and index `idx = bs.traf.id2idx(acid)`, we write the attribute names only for simplification. For example, when we say `approach_bs.intention`, it means `approach_bs.intention[idx]`.

### E1, E2 and D3.  

* When an arrival aircraft (`approach_bs.intention==0`) is leaving the airspace (`approach_bs.prev_under_ctrl==True` and `approach_bs.under_ctrl==False`), if it is in self-navigation mode (`approach_bs.radar_vector==False`), then **reward** it by **$+50$**.

* When a departure aircraft (`approach_bs.intention==1`) is leaving the airspace (`approach_bs.prev_under_ctrl==True` and `approach_bs.under_ctrl==False`), if it is in self-navigation mode (`approach_bs.radar_vector==False`) and it is heading to the designated waypoint (`approachbs.bearing_to_wp-traf.trk` is within $\pm 10$), then **reward** it by **$+50$**. (Note: the difference between $359$ and $1$ are within $\pm 10$.)

* When an aircraft is leaving the airspace (`approach_bs.prev_under_ctrl==True` and `approach_bs.under_ctrl==False`) other than the two cases above, then **penalize** it by **$-100$**.  

### D1.  

* The Loss of Separation (LoS) status of all aircrafts can be accessed by `approach_bs.conflict_detector.inlos`. The previous LoS status of all aircrafts is monitored by `monitor.inlos_prev`. If an in-airspace (`approach_bs.under_ctrl==True`) aircraft become LoS (`monitor.inlos_prev==False` and `approach_bs.conflict_detector.inlos==True`), then **penalize** it by **$-100$**.

* If an in-airspace aircraft is in LoS (`approach_bs.conflict_detector.inlos==True`), then **penalize** it by **$-1$**. (Note: this is a continuous penalty, so the more time it stays in LoS, the more penalty it will get.)

* The one-minute conflict prediction status can be accessed by `approach_bs.conflict_detector.inconf_one_minute`. The duration that each aircraft has `approach_bs.conflict_detector.inconf_one_minute==True` is monitored by `monitor.duration_pred_conf_1m`. In other words, if an aircraft is predicted to have a conflict within one minute and that status has last for `t` seconds, then `monitor.duration_pred_conf_1m` of this aircraft is `t`. For an in-airspace aircraft, if the duration is longer than $5$ seconds, then **penalize** it by **$-0.5$**. (Note: this is a continuous penalty, so the more time it stays in the predicted conflict, the more penalty it will get.)

* The duration that each aircraft has `approach_bs.conflict_detector.inconf_three_minute==True` is monitored by `monitor.duration_pred_conf_3m`. For an in-airspace aircraft, if the duration is longer than $10$ seconds, then **penalize** it by **$-0.1$**. (Note: this is a continuous penalty, so the more time it stays in the predicted conflict, the more penalty it will get.)

### D2.  

* The restricted area invasion status of all aircrafts can be accessed by `approach_bs.invasion_detector.acf_invasion_flag`. The previous restricted area invasion status of all aircrafts is monitored by `monitor.invasion_prev`. If an in-airspace aircraft become in restricted area invasion (`monitor.invasion_prev==False` and `approach_bs.invasion_detector.acf_invasion_flag==True`), then **penalize** it by **$-100$**.  

* If an in-airspace aircraft is in restricted area invasion status (`approach_bs.invasion_detector.acf_invasion_flag==True`), then **penalize** it by **$-1$**. (Note: this is a continuous penalty, so the more time it stays in the restricted area, the more penalty it will get.)  

* The duration that each aircraft has `approach_bs.invasion_detector.acf_invasion_flag_one_minute==True` is monitored by `monitor.duration_pred_inv_1m`. For an in-airspace aircraft, if the duration is longer than $5$ seconds, then **penalize** it by **$-0.5$**. (Note: this is a continuous penalty, so the more time it stays in the predicted invasion, the more penalty it will get.)  

* The duration that each aircraft has `approach_bs.invasion_detector.acf_invasion_flag_three_minute==True` is monitored by `monitor.duration_pred_inv_3m`. For an in-airspace aircraft, if the duration is longer than $10$ seconds, then **penalize** it by **$-0.1$**. (Note: this is a continuous penalty, so the more time it stays in the predicted invasion, the more penalty it will get.)  

### D4.
* The time when the aircrafts enter the airspace can be accessed by `approach_bs.time_entered` (-60.0 if the aircraft has not entered the airspace yet). For an arrival aircraft, if the time it has been in the airspace is longer than `max_in_time`, i.e., `simt-approach_bs.time_entered > max_in_time`, then **penalize** it by **$-0.05$**. (Note: this is a continuous penalty, so the more time it stays in the airspace, the more penalty it will get.)  

* For a departure aircraft, if the time it has been in the airspace is longer than `max_out_time`, i.e., `simt-approach_bs.time_entered > max_out_time`, then **penalize** it by **$-0.05$**. (Note: this is a continuous penalty, so the more time it stays in the airspace, the more penalty it will get.)  

### D5.

* For an arrival aircraft whose action includes a "direct-to" command, if any of the following conditions is satisfied, then **penalize** it by **$-10$**:  
    * `approach_bs.dist_to_wp` is greater than $5$ nautical miles,  
    * The difference between `traf.trk` and the assigned runway course is beyond $\pm 30$ degrees. The assigned runway course of an arrival aircraft with index `idx` can be accessed by
        ```
        apt, rwy = approach_bs.wp_id[idx].split("/")
        rwy_course = approach_bs.tracon.rwy_thres[apt][rwy][2]
        ```  
    * The difference between `traf.alt` and the FAP altitude is less than $-200$ or greater than $500$. The FAP altitude of an arrival aircraft with index `idx` can be accessed by
        ```
        apt, rwy = approach_bs.wp_id[idx].split("/")
        fap_alt = approach_bs.tracon.fap[apt][rwy][2]
        ```  
    * `traf.cas` is greater than $180$ knots.

* For an arrival aircraft whose action includes a "direct-to" command other than the cases above, **reward** it by **$+10$**.  

### D6.

* The time that the aircrafts receive the last command can be accessed by `approach_bs.time_last_cmd`. If `simt - approach_bs.time_last_cmd` is less than `60 / max_actions_per_minute` and the action of the aircraft includes any command, then **penalize** it by **$-0.1$**.  

* If `simt - approach_bs.time_last_cmd` is greater than `60 / min_actions_per_minute` for some aircraft in airspace (`approach_bs.under_ctrl==True`) and such that the aircraft receives no command, then **penalize** it by **$-0.01$**. (Note: this is a continuous penalty, so the more time it stays in the airspace without receiving commands, the more penalty it will get.)  

* If an aircraft out of airspace (`approach_bs.under_ctrl==False`) has an action that includes any command, then **penalize** it by **$-100$**.

### D7.

* If an aircraft is in self-navigation mode (`approach_bs.radar_vector==False`) and its action includes any command, then **penalize** it by **$-1$**.  

* For an in-area aircraft not in self-navigation mode (`approach_bs.under_ctrl==True` and `approach_bs.radar_vector==True`), if it receives a change heading command such that the difference of the new heading and `approach_bs.bearing_to_wp` is within $\pm 30$, then **reward** it by **$0.5$**.

### D8.

* If an aircraft receives a heading change command such that the difference between the new heading and the current track (`traf.trk`) is beyond $\pm 100$ degrees, then **penalize** it by **$-0.5$**.  

### D9.

* If an arrival aircraft receives an altitude change command such that the new altitude is greater than the current altitude, then **penalize** it by **$-0.1$**.  

* If an arrival aircraft receives an altitude change command such that the new altitude is less than the FAP altitude of the assigned runway, then **penalize** it by **$-1$**.  

* If a departure aircraft receives an altitude change command such that the new altitude is less than the current altitude, then **penalize** it by **$-0.1$**.  

* If a departure aircraft receives an altitude change command such that the new altitude is greater than the departure waypoint altitude, then **penalize** it by **$-1$**. Similar to the FAP altitude, the departure waypoint altitude of a departure aircraft with index `idx` can be accessed by
    ```
    wp_alt = approach_bs.tracon.depart_wp[approach_bs.wp_id[idx]][2]
    ```  

### D10.

* If an arrival aircraft receives an airspeed change command such that the new airspeed is greater than the current airspeed, then **penalize** it by **$-0.1$**.  

* If a departure aircraft receives an airspeed change command such that the new airspeed is less than the current airspeed, then **penalize** it by **$-0.1$**.  