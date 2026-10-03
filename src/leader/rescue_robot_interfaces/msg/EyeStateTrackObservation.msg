# Temporary EyeTrack result, not a persistent Survivor Registry observation.
uint8 STATE_INVALID=0
uint8 STATE_OPEN=1
uint8 STATE_CLOSED=2
uint8 STATE_LOW_CONFIDENCE=3
uint8 STATE_NO_PREDICTION=4
uint8 STATE_UNKNOWN=5

uint32 eye_track_id
uint32 face_index
int32 x
int32 y
int32 width
int32 height
float32 face_confidence

bool left_raw_valid
uint8 left_raw_state
float32 left_raw_confidence
uint8 left_stable_state
uint32 left_valid_samples

bool right_raw_valid
uint8 right_raw_state
float32 right_raw_confidence
uint8 right_stable_state
uint32 right_valid_samples

uint8 combined_state
bool history_ready
float32 history_duration_sec
