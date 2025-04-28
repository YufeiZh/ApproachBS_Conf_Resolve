"""
Reward function module for Autonomous TRACON

This module implements the reward function for the reinforcement learning
training of air traffic controllers in TRACON airspace.
"""
import numpy as np
from bluesky.tools.aero import ft, kts


class StatusMonitor:
    """
    Class to monitor the status of aircraft over time
    Used to track changes in aircraft status for reward calculation
    """
    def __init__(self):
        self.id_prev = []
        self.inlos_prev = np.array([], dtype=bool)
        self.invasion_prev = np.array([], dtype=bool)
        self.under_ctrl_prev = np.array([], dtype=bool)
        
        # Duration tracking for conflict and invasion predictions
        self.duration_pred_conf_1m = np.array([])
        self.duration_pred_conf_3m = np.array([])
        self.duration_pred_inv_1m = np.array([])
        self.duration_pred_inv_3m = np.array([])
        
        # Previous time for duration calculations
        self.prev_time = 0.0
    

    def initialize(self, traf):
        """Initialize or resize the monitor arrays based on traffic size"""
        if self.id_prev == traf.id:
            return
        
        inlos_new = np.array([False] * traf.ntraf, dtype=bool)
        invasion_new = np.array([False] * traf.ntraf, dtype=bool)
        under_ctrl_new = np.array([False] * traf.ntraf, dtype=bool)
        duration_c1m_new = np.array([0.0] * traf.ntraf, dtype=float)
        duration_c3m_new = np.array([0.0] * traf.ntraf, dtype=float)
        duration_i1m_new = np.array([0.0] * traf.ntraf, dtype=float)
        duration_i3m_new = np.array([0.0] * traf.ntraf, dtype=float)

        for i, acid in enumerate(traf.id):
            # Check if aircraft is in the airspace
            if acid in self.id_prev:
                idx = self.id_prev.index(acid)
                inlos_new[i] = self.inlos_prev[idx]
                invasion_new[i] = self.invasion_prev[idx]
                under_ctrl_new[i] = self.under_ctrl_prev[idx]
                duration_c1m_new[i] = self.duration_pred_conf_1m[idx]
                duration_c3m_new[i] = self.duration_pred_conf_3m[idx]
                duration_i1m_new[i] = self.duration_pred_inv_1m[idx]
                duration_i3m_new[i] = self.duration_pred_inv_3m[idx]

        self.id_prev = traf.id
        self.inlos_prev = inlos_new
        self.invasion_prev = invasion_new
        self.under_ctrl_prev = under_ctrl_new
        self.duration_pred_conf_1m = duration_c1m_new
        self.duration_pred_conf_3m = duration_c3m_new
        self.duration_pred_inv_1m = duration_i1m_new
        self.duration_pred_inv_3m = duration_i3m_new
        
    
    def update(self, traf, approach_bs, simt):
        """Update the monitored status of aircraft"""
        self.initialize(traf)
        
        # Calculate time difference for duration tracking
        dt = simt - self.prev_time
        self.prev_time = simt
        
        # Update duration tracking for conflicts
        # Reset duration for aircraft no longer in conflict
        self.duration_pred_conf_1m[~approach_bs.conflict_detector.inconf_one_minute] = 0.0
        # Increment duration for aircraft in conflict
        self.duration_pred_conf_1m[approach_bs.conflict_detector.inconf_one_minute] += dt
        
        # Same for 3-minute conflicts
        self.duration_pred_conf_3m[~approach_bs.conflict_detector.inconf_three_minute] = 0.0
        self.duration_pred_conf_3m[approach_bs.conflict_detector.inconf_three_minute] += dt
        
        # Update duration tracking for invasions
        # Reset duration for aircraft no longer in invasion prediction
        self.duration_pred_inv_1m[~approach_bs.invasion_detector.acf_invasion_flag_one_minute] = 0.0
        # Increment duration for aircraft in invasion prediction
        self.duration_pred_inv_1m[approach_bs.invasion_detector.acf_invasion_flag_one_minute] += dt
        
        # Same for 3-minute invasions
        self.duration_pred_inv_3m[~approach_bs.invasion_detector.acf_invasion_flag_three_minute] = 0.0
        self.duration_pred_inv_3m[approach_bs.invasion_detector.acf_invasion_flag_three_minute] += dt
        
        # Store current status for next update
        self.inlos_prev = np.copy(approach_bs.conflict_detector.inlos)
        self.invasion_prev = np.copy(approach_bs.invasion_detector.acf_invasion_flag)
        self.under_ctrl_prev = np.copy(approach_bs.under_ctrl)


class RewardFunction:
    """
    Class implementing the reward function for the TRACON environment
    """
    def __init__(self):
        # Default reward configuration
        self.max_in_time = 1200.0  # 20 minutes
        self.max_out_time = 900.0  # 15 minutes
        self.max_actions_per_minute = 3.0  # Maximum 3 actions per minute
        self.min_actions_per_minute = 0.5  # Minimum 1 action per 2 minutes
    

    def configure(self, max_in_time=None, max_out_time=None,
                 max_actions_per_minute=None, min_actions_per_minute=None):
        """Configure the reward function parameters"""
        if max_in_time is not None:
            self.max_in_time = max_in_time
        if max_out_time is not None:
            self.max_out_time = max_out_time
        if max_actions_per_minute is not None:
            self.max_actions_per_minute = max_actions_per_minute
        if min_actions_per_minute is not None:
            self.min_actions_per_minute = min_actions_per_minute
    

    def calculate_reward(self, traf, simt, approach_bs, monitor, actions):
        """
        Calculate rewards for all aircraft based on their current state and actions
        
        Args:
            traf: BlueSky traffic instance
            simt: Current simulation time (s)
            approach_bs: ApproachBS plugin instance
            actions: Dictionary mapping aircraft IDs to action dictionaries
            
        Returns:
            dict: Dictionary mapping aircraft IDs to reward values
            float: Total reward for all aircraft
        """
        
        # Initialize rewards dictionary
        rewards = {acid: 0.0 for acid in traf.id}
        
        # Calculate rewards for each aircraft
        for i, acid in enumerate(traf.id):
            # Initialize reward components
            reward_components = {}
            
            # E1, E2, D3: Leaving airspace
            reward_components["airspace_exit"] = self._calculate_airspace_exit_reward(i, acid, traf, approach_bs, monitor)
            
            # D1: Aircraft conflicts
            reward_components["conflicts"] = self._calculate_conflict_reward(i, acid, approach_bs, monitor)
            
            # D2: Restricted area invasions
            reward_components["invasions"] = self._calculate_invasion_reward(i, acid, approach_bs, monitor)
            
            # D4: Staying too long in airspace
            reward_components["airspace_time"] = self._calculate_airspace_time_reward(i, simt, approach_bs)
            
            # D5: Direct-to command timing
            reward_components["direct_to"] = self._calculate_direct_to_reward(i, acid, traf, approach_bs, actions)
            
            # D6: Command frequency
            reward_components["cmd_frequency"] = self._calculate_cmd_frequency_reward(i, acid, simt, approach_bs, actions)
            
            # D7: Interrupting self-navigation
            reward_components["self_nav"] = self._calculate_self_nav_reward(i, acid, approach_bs, actions)
            
            # D8: Sharp turns
            reward_components["sharp_turns"] = self._calculate_sharp_turns_reward(i, acid, traf, actions)
            
            # D9: Inappropriate altitude changes
            reward_components["altitude"] = self._calculate_altitude_reward(i, acid, traf, approach_bs, actions)
            
            # D10: Inappropriate speed changes
            reward_components["speed"] = self._calculate_speed_reward(i, acid, traf, approach_bs, actions)
            
            # Sum up all reward components
            rewards[acid] = sum(reward_components.values())
        
        # Calculate total reward
        total_reward = sum(rewards.values())
        
        return rewards, total_reward
    

    def _calculate_airspace_exit_reward(self, idx, acid, traf, approach_bs, monitor):
        """Calculate reward for an aircraft leaving the airspace"""
        reward = 0.0
        
        # Check if aircraft just left the airspace
        prev_under_ctrl = False
        if acid in monitor.id_prev:
            prev_under_ctrl = monitor.under_ctrl_prev[monitor.id_prev.index(acid)]
        if prev_under_ctrl and not approach_bs.under_ctrl[idx]:
            if approach_bs.intention[idx] == 0:  # Arrival aircraft
                # Reward if in self-navigation mode (direct-to was successful)
                if not approach_bs.radar_vector[idx] and approach_bs.dist_to_wp[idx] < 8.0:
                    reward += 50.0
                else:
                    reward -= 100.0  # Penalty for leaving airspace unexpectedly
            elif approach_bs.intention[idx] == 1:  # Departure aircraft
                # Reward if in self-navigation mode and heading to waypoint
                # Calculate heading difference to waypoint
                hdg_diff = abs(approach_bs.bearing_to_wp[idx] % 360 - traf.trk[idx] % 360)
                hdg_diff = min(hdg_diff, 360 - hdg_diff)  # Handle wrap-around
                if not approach_bs.radar_vector[idx] and hdg_diff <= 10:  # Within 10 degrees of the correct heading
                        reward += 50.0
                else:
                    reward -= 100.0  # Penalty for leaving airspace unexpectedly
        
        return reward
    

    def _calculate_conflict_reward(self, idx, acid, approach_bs, monitor):
        """Calculate reward related to conflicts"""
        reward = 0.0
        
        # Only calculate for aircraft in airspace
        if approach_bs.under_ctrl[idx]:
            # Get previous index of the aircraft
            old_index = monitor.id_prev.index(acid) if acid in monitor.id_prev else -1

            # Penalize new Loss of Separation
            inlos_prev = monitor.inlos_prev[old_index] if old_index >= 0 else False
            if not inlos_prev and approach_bs.conflict_detector.inlos[idx]:
                reward -= 100.0
            
            # Continuous penalty for being in LoS
            if approach_bs.conflict_detector.inlos[idx]:
                reward -= 1.0
            
            # Penalty for predicted conflicts (1 minute)
            duration = monitor.duration_pred_conf_1m[old_index] if old_index >= 0 else 0.0
            if approach_bs.conflict_detector.inconf_one_minute[idx] and duration > 5.0:
                reward -= 0.5
            
            # Penalty for predicted conflicts (3 minutes)
            duration = monitor.duration_pred_conf_3m[old_index] if old_index >= 0 else 0.0
            if approach_bs.conflict_detector.inconf_three_minute[idx] and monitor.duration_pred_conf_3m[idx] > 10.0:
                reward -= 0.1
        
        return reward
    

    def _calculate_invasion_reward(self, idx, acid, approach_bs, monitor):
        """Calculate reward related to restricted area invasions"""
        reward = 0.0
        
        # Only calculate for aircraft in airspace
        if approach_bs.under_ctrl[idx]:
            # Get previous index of the aircraft
            old_index = monitor.id_prev.index(acid) if acid in monitor.id_prev else -1

            # Penalize new invasion
            invasion_prev = monitor.invasion_prev[old_index] if old_index >= 0 else False
            if not invasion_prev and approach_bs.invasion_detector.acf_invasion_flag[idx]:
                reward -= 100.0
            
            # Continuous penalty for being in restricted area
            if approach_bs.invasion_detector.acf_invasion_flag[idx]:
                reward -= 1.0
            
            # Penalty for predicted invasions (1 minute)
            duration = monitor.duration_pred_inv_1m[old_index] if old_index >= 0 else 0.0
            if approach_bs.invasion_detector.acf_invasion_flag_one_minute[idx] and duration > 5.0:
                reward -= 0.5
            
            # Penalty for predicted invasions (3 minutes)
            duration = monitor.duration_pred_inv_3m[old_index] if old_index >= 0 else 0.0
            if approach_bs.invasion_detector.acf_invasion_flag_three_minute[idx] and duration > 10.0:
                reward -= 0.1
        
        return reward
    
    
    def _calculate_airspace_time_reward(self, idx, simt, approach_bs):
        """Calculate reward for time spent in airspace"""
        reward = 0.0
        
        # Only calculate for aircraft in airspace
        if approach_bs.under_ctrl[idx] and approach_bs.time_entered[idx] > 0:
            time_in_airspace = simt - approach_bs.time_entered[idx]
            
            # Penalize staying too long based on aircraft type
            if approach_bs.intention[idx] == 0:  # Arrival
                if time_in_airspace > self.max_in_time:
                    reward -= 0.05
            elif approach_bs.intention[idx] == 1:  # Departure
                if time_in_airspace > self.max_out_time:
                    reward -= 0.05
        
        return reward
    

    def _calculate_direct_to_reward(self, idx, acid, traf, approach_bs, actions):
        """Calculate reward for direct-to commands"""
        reward = 0.0
        
        # Check if this aircraft received a direct-to command
        if acid in actions and actions[acid]['send_command'] and actions[acid]['direct_to']:
            # Only evaluate for arrival aircraft
            if approach_bs.intention[idx] == 0:
                # Get runway information
                apt, rwy = approach_bs.wp_id[idx].split("/")
                rwy_course = approach_bs.tracon.rwy_thres[apt][rwy][2]
                fap_alt = approach_bs.tracon.fap[apt][rwy][2]
                
                # Check conditions for appropriate direct-to
                inappropriate = False
                
                # Too far from waypoint
                if approach_bs.dist_to_wp[idx] > 5.0:
                    inappropriate = True
                
                # Wrong heading
                hdg_diff = abs(traf.trk[idx] % 360 - rwy_course % 360)
                hdg_diff = min(hdg_diff, 360 - hdg_diff)  # Handle wrap-around
                if hdg_diff > 30:
                    inappropriate = True
                
                # Wrong altitude
                alt_diff = traf.alt[idx] / ft - fap_alt
                if alt_diff < -200 or alt_diff > 500:
                    inappropriate = True
                
                # Too fast
                if traf.cas[idx] / kts > 180:
                    inappropriate = True
                
                # Apply reward or penalty
                if inappropriate:
                    reward -= 10.0
                else:
                    reward += 10.0
        
        return reward
    

    def _calculate_cmd_frequency_reward(self, idx, acid, simt, approach_bs, actions):
        """Calculate reward related to command frequency"""
        reward = 0.0
        
        # Penalty for commands that are too frequent
        min_time_between_cmds = 60.0 / self.max_actions_per_minute
        if acid in actions and actions[acid]['send_command']:
            time_since_last_cmd = simt - approach_bs.time_last_cmd[idx]
            if time_since_last_cmd < min_time_between_cmds:
                reward -= 0.1
        
        # Penalty for not sending commands frequently enough
        if approach_bs.under_ctrl[idx]:
            min_cmd_frequency = 60.0 / self.min_actions_per_minute
            time_since_last_cmd = simt - approach_bs.time_last_cmd[idx]
            if time_since_last_cmd > min_cmd_frequency and (acid not in actions or not actions[acid]['send_command']):
                reward -= 0.01
        
        # Severe penalty for sending commands to aircraft not in airspace
        if not approach_bs.under_ctrl[idx] and acid in actions and actions[acid]['send_command']:
            reward -= 100.0

        # Penalty for sending empty commands
        if acid in actions and actions[acid]['send_command'] and not actions[acid]['direct_to'] and \
           not actions[acid]['change_heading'] and not actions[acid]['change_altitude'] and \
           not actions[acid]['change_speed']:
            reward -= 10.0
        
        return reward
    
    
    def _calculate_self_nav_reward(self, idx, acid, approach_bs, actions):
        """Calculate reward related to interrupting self-navigation"""
        reward = 0.0
        
        # Penalty for interrupting self-navigation
        if not approach_bs.radar_vector[idx] and acid in actions and actions[acid]['send_command']:
            reward -= 1.0
        
        # Reward for good heading commands that align with waypoint
        if approach_bs.under_ctrl[idx] and approach_bs.radar_vector[idx]:
            if acid in actions and actions[acid]['send_command'] and not actions[acid]['direct_to']:
                # Compare new heading with bearing to waypoint
                new_hdg = actions[acid]['heading']
                hdg_diff = abs(new_hdg % 360 - approach_bs.bearing_to_wp[idx] % 360)
                hdg_diff = min(hdg_diff, 360 - hdg_diff)  # Handle wrap-around
                
                if hdg_diff <= 30:  # Within 30 degrees of bearing to waypoint
                    reward += 0.5
        
        return reward
    

    def _calculate_sharp_turns_reward(self, idx, acid, traf, actions):
        """Calculate reward related to sharp turns"""
        reward = 0.0
        
        # Penalty for sharp turns
        if acid in actions and actions[acid]['send_command'] and not actions[acid]['direct_to'] and \
           actions[acid]['change_heading']:
            
            new_hdg = actions[acid]['heading']
            current_trk = traf.trk[idx]
            
            hdg_diff = abs(new_hdg % 360 - current_trk % 360)
            hdg_diff = min(hdg_diff, 360 - hdg_diff)  # Handle wrap-around
            
            if hdg_diff > 100:  # More than 100 degrees turn
                reward -= 0.5
        
        return reward
    

    def _calculate_altitude_reward(self, idx, acid, traf, approach_bs, actions):
        """Calculate reward related to altitude changes"""
        reward = 0.0
        
        # Check if this aircraft received an altitude change command
        if acid in actions and actions[acid]['send_command'] and not actions[acid]['direct_to'] and \
           actions[acid]['change_altitude']:
            new_alt = actions[acid]['altitude']
            current_alt = traf.alt[idx] / ft
            
            # Check aircraft intention
            if approach_bs.intention[idx] == 0:  # Arrival
                # Penalty for climbing
                if new_alt > current_alt:
                    reward -= 0.1
                
                # Penalty for descending below FAP altitude
                apt, rwy = approach_bs.wp_id[idx].split("/")
                fap_alt = approach_bs.tracon.fap[apt][rwy][2]
                if new_alt < fap_alt:
                    reward -= 1.0
                    
            elif approach_bs.intention[idx] == 1:  # Departure
                # Penalty for descending
                if new_alt < current_alt:
                    reward -= 0.1
                
                # Penalty for climbing above waypoint altitude
                wp_alt = approach_bs.tracon.depart_wp[approach_bs.wp_id[idx]][2]
                if new_alt > wp_alt:
                    reward -= 1.0
        
        return reward


    def _calculate_speed_reward(self, idx, acid, traf, approach_bs, actions):
        """Calculate reward related to speed changes"""
        reward = 0.0
        
        # Check if this aircraft received a speed change command
        if acid in actions and actions[acid]['send_command'] and not actions[acid]['direct_to'] and \
           actions[acid]['change_speed']:
            new_spd = actions[acid]['speed']
            current_spd = traf.cas[idx] / kts
            
            # Check aircraft intention
            if approach_bs.intention[idx] == 0:  # Arrival
                # Penalty for accelerating
                if new_spd > current_spd:
                    reward -= 0.1
                    
            elif approach_bs.intention[idx] == 1:  # Departure
                # Penalty for decelerating
                if new_spd < current_spd:
                    reward -= 0.1
        
        return reward


def get_reward_function():
    """Factory function to create and return a RewardFunction instance"""
    return RewardFunction()