bool ready               # true if enabled and in the home position
bool enabled             # true if enabled
bool error               # true if a component of the assembly has an error
#
# The following are specific to the canbus state, this is enmu sturct.
uint8  canbus_state      # One of the following:
  uint8   CANBUS_STATE_NORMAL = 0  
  uint8   CANBUS_STATE_INTERRUPT = 1  # The canbus interruption occurs in any joint module 

# Save the joint ID that the canbus interrupts.
int32[] interruptJoints
