float64 timestamp              # time
float64 time_resolution        # timestamp between trajectory points

int16 	go                     # default: 1
int16 	estop                  # default: 0

float64[] x_vector             # meters, 150 points maximum
float64[] y_vector             # meters, 150 points maximum
float64[] vd_vector            # meters, 150 points maximum
float64[] ori_vector           # radians, 150 points maximum
