# One frame-local face observation. face_index is not an EyeTrack or Survivor ID.
uint8 STATE_INVALID=0
uint8 STATE_OPEN=1
uint8 STATE_CLOSED=2
uint8 STATE_LOW_CONFIDENCE=3
uint8 STATE_NO_PREDICTION=4

uint32 face_index
int32 x
int32 y
int32 width
int32 height
float32 face_confidence

bool left_valid
uint8 left_state
float32 left_confidence
bool right_valid
uint8 right_state
float32 right_confidence
