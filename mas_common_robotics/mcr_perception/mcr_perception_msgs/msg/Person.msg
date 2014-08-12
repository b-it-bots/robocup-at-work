Header header                   # standard header

uint32 id						# unique identifier to a person
	
geometry_msgs/PoseStamped pose	# real pose of a detected person

geometry_msgs/PoseStamped safe_pose		# safety pose for a detected person (to approach by base)

float32 height					# measured height of the person

float32 width					# measured width of the person 

float32 depth					# measured depth of the person

bool is_tracked					# is the user tracked

bool is_occluded				# is the user occluded 

float32 probability				# probability of beeing a person at this pose
