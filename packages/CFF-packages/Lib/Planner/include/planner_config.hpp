
namespace Planner
{

struct PlannerConfig
{
    float maxSpeed;
    float minSpeed;
    float maxYawrate;
    float maxAccel;
    float maxdYawrate;
    float velocityResolution;
    float yawrateResolution;
    float dt;
    float predictTime;
    float heading;
    float clearance;
    float velocity;
    Rect base;
};

}