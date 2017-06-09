# pregrasp_pose and object ARE NOT manifolds but specific samples 
int32 PREGRASP_DISK = 0
int32 PREGRASP_CYLINDER = 1
int32 PREGRASP_SPHERE = 2
int32 PREGRASP_HOOK = 3
int32 PREGRASP_BOX = 4

int32 STRATEGY_PUSH = 0
int32 STRATEGY_APPROACH_THEN_PUSH = 1
int32 STRATEGY_SQUEEZE = 10
int32 STRATEGY_APPROACH_THEN_SQUEEZE = 11
int32 STRATEGY_SLIDE_TO_EDGE = 20
int32 STRATEGY_LAND = 30
int32 STRATEGY_POSITION = 40
int32 STRATEGY_WALL_GRASP = 50

int32 id
int32 strategy
int32 pregrasp_configuration
PreGraspManifold pregrasp_pose
PreGraspManifold object
float64 quality_grasp
float64 quality_approach
float64 quality_closing
