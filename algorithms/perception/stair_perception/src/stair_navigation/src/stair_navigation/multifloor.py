class MultiFloorLifecycle:
    """Pure coordination state; owns no motion, mapping, or planning."""
    STATES = ('EXPLORE', 'PAUSE_EXPLORER', 'STAIR_APPROACH', 'STAIR_TRAVERSE',
              'FLOOR_HANDOFF', 'WAIT_FLOOR_MAP', 'RESET_EXPLORER')

    def __init__(self, floor_id=0, session_id=0):
        self.state = 'EXPLORE'
        self.floor_id = floor_id
        self.session_id = session_id
        self.transition_floor = None
        self.expected_session = None
        self.transition_count = 0

    def exploration_complete(self, stair_available):
        if self.state != 'EXPLORE' or not stair_available:
            return False
        self.state = 'PAUSE_EXPLORER'
        self.transition_floor = self.floor_id
        self.transition_count += 1
        return True

    def explorer_paused(self, target_invalid):
        if self.state != 'PAUSE_EXPLORER' or not target_invalid:
            return False
        self.state = 'STAIR_APPROACH'
        return True

    def approach_handoff(self):
        if self.state != 'STAIR_APPROACH':
            return False
        self.state = 'STAIR_TRAVERSE'
        return True

    def stair_complete(self):
        if self.state != 'STAIR_TRAVERSE':
            return False
        self.state = 'FLOOR_HANDOFF'
        return True

    def observe_floor(self, floor_id):
        if self.state != 'FLOOR_HANDOFF' or floor_id <= self.floor_id:
            return False
        self.floor_id = floor_id
        self.state = 'WAIT_FLOOR_MAP'
        return True

    def floor_ready(self, map_fresh, pose_valid, stationary):
        if self.state != 'WAIT_FLOOR_MAP' or not (map_fresh and pose_valid and stationary):
            return False
        self.expected_session = self.session_id + 1
        self.state = 'RESET_EXPLORER'
        return True

    def explorer_reset(self, session_id, complete_cleared):
        if (self.state != 'RESET_EXPLORER' or session_id != self.expected_session or
                not complete_cleared):
            return False
        self.session_id = session_id
        self.expected_session = None
        self.transition_floor = None
        self.state = 'EXPLORE'
        return True
