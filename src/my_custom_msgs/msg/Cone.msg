# A message for representing a detected cone

# Position of the cone in 3D space
float32 x
float32 y
float32 z

# Classification of the cone, for example 0 for unknown, 1 for blue, 2 for yellow
uint8 UNKNOWN = 0
uint8 BLUE = 1
uint8 YELLOW = 2
uint8 classification
