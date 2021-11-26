
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>


using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
namespace {
    constexpr int SLOW_DAWN_TRESHOLD = 1800;
    
    constexpr int BOOST_DISTANCE_TRESHOLD = 4000;
    constexpr int LONG_ROUTE_TRESHOLD = 4000;
    
    constexpr float ANGLE_TRESHOLD = 80.0f/90.0f;
    constexpr int MAX_THRUST = 100;
    constexpr int MIN_THRUST = 60;
    constexpr int MIN_PTHRUST = 30;
    constexpr int PREV_SLOW_DAWN_TRESHOLD = 1500;
    constexpr int COLLISION_TRESHOLD = 2500;
    constexpr int PLAYER_COLLISION_TRESHOLD = 2100;
    constexpr float CHECKPOINT_RADIUS = 600;
    constexpr float POD_RADIUS = 400;

    constexpr int COLLISIONS_ANGLE_TRESHOLD = 15;
    constexpr float PI = 3.14159265359f;
    constexpr float SHOULD_SLOWDAWN_ANGLE_TRESHOLD = 60.0f;
    constexpr int SIMULATION_LEVEL = 4;
    constexpr int OPPONENT_SIMULATION_LEVEL = 2;
    constexpr float SPEED_TO_SHIELD = 120.0f;
    constexpr float DOT_TO_SHIELD = 0.8f;
    constexpr float BOOST_ANGLE_TRESHOLD = 10.0f * PI/180.0f;
    
}


class Vector2D
{ 
public:
    Vector2D(int xx, int yy)
    {
        x = xx;
        y = yy;
    }
    Vector2D(float xx = 0, float yy = 0)
    {
        x = xx;
        y = yy;
    }
    ~Vector2D() = default;
 
    void Rotate( const float angle )
    {
        float xt = (x * cosf(angle)) - (y * sinf(angle));
        float yt = (y * cosf(angle)) + (x * sinf(angle));
        x = xt;
        y = yt;
    };
    float Magnitude() const
    {
        return sqrtf(x * x + y * y);
    };
    float getAngle() const
    {
        float mag = Magnitude();
        float angle = acos( x/mag );
        if(y < 0.0f)
        {
            angle = 2.0f* PI - angle;
        }
        return angle;
    }

    float Normalize()
    {
        float mag = Magnitude();
 
        if(mag != 0.0f)
        {
            x /= mag;
            y /= mag;
        }
 
        return mag;

    };

    float DotProduct( const Vector2D& v2 ) const
    {
        return (x * v2.x) + (y * v2.y);
    };

    float CrossProduct( const Vector2D& v2 ) const
    {
        return (x * v2.y) - (y * v2.x);
    };
 
    static Vector2D Zero()
    {
        return Vector2D(0, 0);
    };

    static float Distance( const Vector2D& v1, const Vector2D& v2)
    {
        return sqrtf( pow((v2.x - v1.x), 2 ) + pow((v2.y - v1.y), 2) );
    };
 
    Vector2D& operator= ( const Vector2D& v2 )
    {
        if (this == &v2)
            return *this;
 
        x = v2.x;
        y = v2.y;
    
        return *this;
    };
 
    Vector2D& operator+= ( const Vector2D& v2 )
    {        
        x += v2.x;
        y += v2.y;
    
        return *this;
    };

    Vector2D& operator-= ( const Vector2D& v2 )
    {
        
        x -= v2.x;
        y -= v2.y;
    
        return *this;
    };

    Vector2D& operator*= ( const float scalar)
    {
        x *= scalar;
        y *= scalar;
 
        return *this;
    };

    Vector2D& operator/= ( const float scalar)
    {
        x /= scalar;
        y /= scalar;
    
        return *this;
    };
 
    const Vector2D operator+( const Vector2D &v2 ) const
    {
        return Vector2D(*this) += v2;
    };

    const Vector2D operator-( const Vector2D &v2 ) const
    {
        return Vector2D(*this) -= v2;
    };

    const Vector2D operator*( const float scalar ) const
    {
        return Vector2D(*this) *= scalar;
    };

    const Vector2D operator/( const float scalar ) const
    {
        return Vector2D(*this) /= scalar;
    };
 
    bool operator== ( const Vector2D& v2 ) const
    {
        return ((x == v2.x) && (y == v2.y));
    };

    bool operator!= ( const Vector2D& v2 ) const
    {
        return !((x == v2.x) && (y == v2.y));
    };
 
public:
    float x, y;
};


class PodData
{
    
       
    public:
        PodData(const Vector2D& pos, const Vector2D& s, const int id, const float a ) :
            position(pos), speed(s), check_point_id(id), angle(a), lap(0),
            passed_checkpoint(false)
        {            
        }

        PodData(){};
        PodData(const PodData&) = default;

        void UpdatePod(const Vector2D& pos, const Vector2D& s, const int a, const int ch_point_id)
        {
            position = pos;
            speed = s;
            if(check_point_id != ch_point_id && ch_point_id==0)
            {
                lap++;
            }
            check_point_id = ch_point_id;
            angle = a;
            

                        
        }

        void addAngle(float angleDiff){
            angle += angleDiff;
            //cerr<< " angle = a " << angle << endl;
            if (angle < 0.0f) {
                angle = 360.0f + angle;
            } else if (angle > 360.0f) {
                angle = angle - 360.0f;
            }
        }
        //used for simulation - moves the pod
        //following expert's rules
        float move(float thrust, float angleDiff, bool debug = false) 
        {
            Vector2D prevpos(position);
            addAngle(angleDiff);
            //if(debug)
            //    cerr<< " angleDiff " << angleDiff << " speed.x "<< speed.x << endl;
            float rangle = angle * (PI / 180.0f);
           // if(debug)
             //   cerr<< " cos(rangle) " << cos(rangle) << endl;
            speed += Vector2D((float)thrust * cos(rangle), (float)thrust * sin(rangle));
            if(debug){
               // cerr<< " position.x " << position.x << endl;
               // cerr<< " position.y " << position.y << endl;
            }
            position+=speed;
            int px = (int)position.x;
            int py = (int)position.y;
            position = Vector2D(px, py);
            if(debug){
                //cerr<< " after position.x " << position.x << endl;
                //cerr<< " after position.y " << position.y << endl;
            }
            float mag = (position - prevpos).Magnitude();
            //friction
            speed *= 0.85f;


            int sx = (int)speed.x;
            int sy = (int)speed.y;
            speed = Vector2D(sx, sy);
            return mag;
             
        }

        void setTargetCheckpoint(int tcheckpoint)
        {
            target_checkpoint = tcheckpoint;
        }

        bool Collide(const Vector2D& pos, const float collision_radius)
        {
            Vector2D offset = position-pos;
            //cerr<< " offset.Magnitude() "<< offset.Magnitude() << endl;
            return  offset.Magnitude() < (collision_radius + POD_RADIUS);
        }
        
    public:
        Vector2D position;
        Vector2D speed;
        int check_point_id;
        float angle;
        int lap;
        int target_checkpoint;
        bool passed_checkpoint;
        
};

struct OutputValues{
    int out_x;
    int out_y;
    string sthrust;
};


class Game
{
    private:
        
        vector<PodData> pods;
        vector<Vector2D> checkpoints;
        vector<bool> should_slowdawn;
        int laps;
        int nr_check_points;
        bool has_boost;
        int longest_route_id;
        
    public:
    Game(const vector<Vector2D>& chpoints, int l) :
        checkpoints(chpoints) , laps(l), has_boost(true)
    {      
        pods.resize(4);
        nr_check_points = checkpoints.size();
        
        longest_route_id = getLongestRouteID();
        cerr<< "longest_route_id " << longest_route_id<< endl;
        ComputeShouldSlowdawn();
    }
    //returns closest point in checkpoint depending on players position 
    //and orientation
    Vector2D getCheckpointPosFromPlayer(const Vector2D& pos, const float& angle,
         const Vector2D& pos_checkpoint)
    {
        Vector2D offset = pos_checkpoint - pos;
        float angle_diff = abs(offset.getAngle()*180.0f/PI - angle);
        if(angle_diff > 90.0f)
        {
            return pos_checkpoint;
        }
        //player direction
        Vector2D dir(cos(angle*PI/180.0f), sin(angle*PI/180.0f));
        //check if dir intersect circle
        


    }


    //returns distance evaluation
    float PickThrustAngleHelper(const PodData& pod,  int& pthrust, const Vector2D& target,
         const vector<float>& angles,  const vector<float>& thrusts, int level, bool chaser, bool debug = false)
    {
        

        Vector2D offset = target -  pod.position;
        
        float dist = offset.Magnitude();
        if(level == 0)// || dist < 2.0f* CHECKPOINT_RADIUS)
        {
            offset.Normalize();
            Vector2D dir(cos(pod.angle *PI/180.0f ),  sin(pod.angle *PI/180.0f ));  
            
            //cerr<< " offset.x " << offset.x << endl;
            //cerr<< " offset.y " << offset.y << endl;
            float angle = abs(acos(offset.DotProduct(dir)));
            if(debug){
                //cerr<< " pod.angle " << pod.angle << " " << angle << " dist " << dist << endl;
                //cerr<< " pthrust " << pthrust << " pthrust*angle*angle " << pthrust*angle*angle<<endl;
            }
            if(pod.passed_checkpoint)
            {
                return dist + pod.speed.Magnitude()*angle*angle * 10.0f;
            } else {
                return dist;
            }
        }
        
        float min_dist = numeric_limits<float>::max();
        int picked_thrust;
        bool passed_checkpoint = pod.passed_checkpoint;
        for(int cangle = 0;  cangle<angles.size(); ++cangle)
        {
            for(int cthrust = 0;  cthrust<thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                //cerr<< " pod.angle " << pod.angle<<endl;
                
                int th  = thrusts[cthrust];
                float dist = p.move(thrusts[cthrust], angles[cangle], false);
                Vector2D t(target);
                if(!chaser){
                    if(p.passed_checkpoint)
                    {
                        t = checkpoints[(pod.check_point_id + 1)%nr_check_points];
                      //  cerr<< "checked 2 " << endl; 
                    } else {
                        t = checkpoints[pod.check_point_id];
                    }
                    
                    float dist_eval =  PickThrustAngleHelper(p, th, t, 
                        angles, thrusts, level-1, chaser, cangle==0 );
                    if(!passed_checkpoint && p.passed_checkpoint)
                    {
                    
                        passed_checkpoint = true;
                        picked_thrust = thrusts[cthrust];
                        min_dist = dist_eval;
                        
                        
                    }else if(min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                    {
                        min_dist = dist_eval;            
                        picked_thrust = thrusts[cthrust];            
                        //if(cangle == 0)
                        //cerr<< " best_pod.angle  " << best_pod.angle << endl; 
                        //cerr<< " best_thrust  " << best_thrust << endl;

                    }
                } else{
                    float dist_eval =  PickThrustAngleHelper(p, th, t, angles, thrusts, level-1, chaser, debug);
                    if(min_dist > dist_eval)
                    {
                        //if(debug)
                        //    cerr<< " dist_eval " << dist_eval<<endl;
                        min_dist = dist_eval;
                        //cerr<< " min_dist " << min_dist<<endl;
                                
                    }
                }

                
                //cerr<< " p.angle " << p.angle << " cthrust " << cthrust << 
                //" dist "<< dist << " dist_eval " << dist_eval<< endl;
               
                                
            }
        }
        if(passed_checkpoint)
        {
            min_dist -=1000000.0f;
        }
        if(level == SIMULATION_LEVEL)
        {
            //pthrust = picked_thrust;
            //cerr<<"pthrust "<< pthrust<<  " picked_thrust " <<picked_thrust<<endl;
        }
        return min_dist;

    }

    //find best target and thrust by simulating real movements 
    void PickThrustAngle(OutputValues& out_pod, const PodData& pod, const int& level)
    {
        //vector<float> angles = { -18.0f * PI/180.0f, -9.0f * PI/180.0f, -2.0f* PI/180.0f, 0.0f, 2.0f* PI/180.0f, 
        //    9.0f*PI/180.0f, 18.0f * PI/180.0f };
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f  };
        vector<float> angles1 = { -18.0f , 0.0f, 18.0f  };
        //vector<float> sinuses = { sin(-18.0f * PI/180.0f), sin(-9.0f * PI/180.0f), sin(-2.0f* PI/180.0f), sin(0.0f), sin(2.0f* PI/180.0f), 
        //    sin(9.0f*PI/180.0f), sin(18.0f * PI/180.0f) };
        //vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        vector<float> thrusts = { 55.0f, 100.0f };
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>


using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/
namespace {
    constexpr int SLOW_DAWN_TRESHOLD = 1800;
    
    constexpr int BOOST_DISTANCE_TRESHOLD = 4000;
    constexpr int LONG_ROUTE_TRESHOLD = 4000;
    
    constexpr float ANGLE_TRESHOLD = 80.0f/90.0f;
    constexpr int MAX_THRUST = 100;
    constexpr int MIN_THRUST = 60;
    constexpr int MIN_PTHRUST = 30;
    constexpr int PREV_SLOW_DAWN_TRESHOLD = 1500;
    constexpr int COLLISION_TRESHOLD = 2500;
    constexpr int PLAYER_COLLISION_TRESHOLD = 2100;
    constexpr float CHECKPOINT_RADIUS = 600;
    constexpr float POD_RADIUS = 400;

    constexpr int COLLISIONS_ANGLE_TRESHOLD = 15;
    constexpr float PI = 3.14159265359f;
    constexpr float SHOULD_SLOWDAWN_ANGLE_TRESHOLD = 60.0f;
    constexpr int SIMULATION_LEVEL = 4;
    constexpr int OPPONENT_SIMULATION_LEVEL = 2;
    constexpr float SPEED_TO_SHIELD = 120.0f;
    constexpr float DOT_TO_SHIELD = 0.8f;
    constexpr float BOOST_ANGLE_TRESHOLD = 10.0f * PI/180.0f;
    
}


class Vector2D
{ 
public:
    Vector2D(int xx, int yy)
    {
        x = xx;
        y = yy;
    }
    Vector2D(float xx = 0, float yy = 0)
    {
        x = xx;
        y = yy;
    }
    ~Vector2D() = default;
 
    void Rotate( const float angle )
    {
        float xt = (x * cosf(angle)) - (y * sinf(angle));
        float yt = (y * cosf(angle)) + (x * sinf(angle));
        x = xt;
        y = yt;
    };
    float Magnitude() const
    {
        return sqrtf(x * x + y * y);
    };
    float getAngle() const
    {
        float mag = Magnitude();
        float angle = acos( x/mag );
        if(y < 0.0f)
        {
            angle = 2.0f* PI - angle;
        }
        return angle;
    }

    float Normalize()
    {
        float mag = Magnitude();
 
        if(mag != 0.0f)
        {
            x /= mag;
            y /= mag;
        }
 
        return mag;

    };

    float DotProduct( const Vector2D& v2 ) const
    {
        return (x * v2.x) + (y * v2.y);
    };

    float CrossProduct( const Vector2D& v2 ) const
    {
        return (x * v2.y) - (y * v2.x);
    };
 
    static Vector2D Zero()
    {
        return Vector2D(0, 0);
    };

    static float Distance( const Vector2D& v1, const Vector2D& v2)
    {
        return sqrtf( pow((v2.x - v1.x), 2 ) + pow((v2.y - v1.y), 2) );
    };
 
    Vector2D& operator= ( const Vector2D& v2 )
    {
        if (this == &v2)
            return *this;
 
        x = v2.x;
        y = v2.y;
    
        return *this;
    };
 
    Vector2D& operator+= ( const Vector2D& v2 )
    {        
        x += v2.x;
        y += v2.y;
    
        return *this;
    };

    Vector2D& operator-= ( const Vector2D& v2 )
    {
        
        x -= v2.x;
        y -= v2.y;
    
        return *this;
    };

    Vector2D& operator*= ( const float scalar)
    {
        x *= scalar;
        y *= scalar;
 
        return *this;
    };

    Vector2D& operator/= ( const float scalar)
    {
        x /= scalar;
        y /= scalar;
    
        return *this;
    };
 
    const Vector2D operator+( const Vector2D &v2 ) const
    {
        return Vector2D(*this) += v2;
    };

    const Vector2D operator-( const Vector2D &v2 ) const
    {
        return Vector2D(*this) -= v2;
    };

    const Vector2D operator*( const float scalar ) const
    {
        return Vector2D(*this) *= scalar;
    };

    const Vector2D operator/( const float scalar ) const
    {
        return Vector2D(*this) /= scalar;
    };
 
    bool operator== ( const Vector2D& v2 ) const
    {
        return ((x == v2.x) && (y == v2.y));
    };

    bool operator!= ( const Vector2D& v2 ) const
    {
        return !((x == v2.x) && (y == v2.y));
    };
 
public:
    float x, y;
};


class PodData
{
    
       
    public:
        PodData(const Vector2D& pos, const Vector2D& s, const int id, const float a ) :
            position(pos), speed(s), check_point_id(id), angle(a), lap(0),
            passed_checkpoint(false)
        {            
        }

        PodData(){};
        PodData(const PodData&) = default;

        void UpdatePod(const Vector2D& pos, const Vector2D& s, const int a, const int ch_point_id)
        {
            position = pos;
            speed = s;
            if(check_point_id != ch_point_id && ch_point_id==0)
            {
                lap++;
            }
            check_point_id = ch_point_id;
            angle = a;
            

                        
        }

        void addAngle(float angleDiff){
            angle += angleDiff;
            //cerr<< " angle = a " << angle << endl;
            if (angle < 0.0f) {
                angle = 360.0f + angle;
            } else if (angle > 360.0f) {
                angle = angle - 360.0f;
            }
        }
        //used for simulation - moves the pod
        //following expert's rules
        float move(float thrust, float angleDiff, bool debug = false) 
        {
            Vector2D prevpos(position);
            addAngle(angleDiff);
            //if(debug)
            //    cerr<< " angleDiff " << angleDiff << " speed.x "<< speed.x << endl;
            float rangle = angle * (PI / 180.0f);
           // if(debug)
             //   cerr<< " cos(rangle) " << cos(rangle) << endl;
            speed += Vector2D((float)thrust * cos(rangle), (float)thrust * sin(rangle));
            if(debug){
               // cerr<< " position.x " << position.x << endl;
               // cerr<< " position.y " << position.y << endl;
            }
            position+=speed;
            int px = (int)position.x;
            int py = (int)position.y;
            position = Vector2D(px, py);
            if(debug){
                //cerr<< " after position.x " << position.x << endl;
                //cerr<< " after position.y " << position.y << endl;
            }
            float mag = (position - prevpos).Magnitude();
            //friction
            speed *= 0.85f;


            int sx = (int)speed.x;
            int sy = (int)speed.y;
            speed = Vector2D(sx, sy);
            return mag;
             
        }

        void setTargetCheckpoint(int tcheckpoint)
        {
            target_checkpoint = tcheckpoint;
        }

        bool Collide(const Vector2D& pos, const float collision_radius)
        {
            Vector2D offset = position-pos;
            //cerr<< " offset.Magnitude() "<< offset.Magnitude() << endl;
            return  offset.Magnitude() < (collision_radius + POD_RADIUS);
        }
        
    public:
        Vector2D position;
        Vector2D speed;
        int check_point_id;
        float angle;
        int lap;
        int target_checkpoint;
        bool passed_checkpoint;
        
};

struct OutputValues{
    int out_x;
    int out_y;
    string sthrust;
};


class Game
{
    private:
        
        vector<PodData> pods;
        vector<Vector2D> checkpoints;
        vector<bool> should_slowdawn;
        int laps;
        int nr_check_points;
        bool has_boost;
        int longest_route_id;
        
    public:
    Game(const vector<Vector2D>& chpoints, int l) :
        checkpoints(chpoints) , laps(l), has_boost(true)
    {      
        pods.resize(4);
        nr_check_points = checkpoints.size();
        
        longest_route_id = getLongestRouteID();
        cerr<< "longest_route_id " << longest_route_id<< endl;
        ComputeShouldSlowdawn();
    }
    //returns closest point in checkpoint depending on players position 
    //and orientation
    Vector2D getCheckpointPosFromPlayer(const Vector2D& pos, const float& angle,
         const Vector2D& pos_checkpoint)
    {
        Vector2D offset = pos_checkpoint - pos;
        float angle_diff = abs(offset.getAngle()*180.0f/PI - angle);
        if(angle_diff > 90.0f)
        {
            return pos_checkpoint;
        }
        
        //check if dir intersect circle
        if( offset.Magnitude() * sin(angle_diff*PI/180.0f) < CHECKPOINT_RADIUS)
        {
            return pos_checkpoint;
        }
        //player direction
        Vector2D dir(cos(angle*PI/180.0f), sin(angle*PI/180.0f));
        float dot = dir.DotProduct(offset);
        float diff_outside = (offset.Magnitude() * sin(angle_diff*PI/180.0f) - (float)CHECKPOINT_RADIUS);
        float angle_outside = atan( diff_outside/dot );
        float angle_diff1 = abs(angle_outside - angle *PI/180.0f);
        float dist = sqrt(offset.Magnitude()*offset.Magnitude() - ((float)CHECKPOINT_RADIUS-2)*((float)CHECKPOINT_RADIUS-2)); 
        return Vector2D(dist * cos(angle_diff1), dist * sin(angle_diff1));
    }


    //returns distance evaluation
    float PickThrustAngleHelper(const PodData& pod,  int& pthrust, const Vector2D& target,
         const vector<float>& angles,  const vector<float>& thrusts, int level, bool chaser, bool debug = false)
    {
        

        Vector2D offset = target -  pod.position;
        
        float dist = offset.Magnitude();
        if(level == 0)// || dist < 2.0f* CHECKPOINT_RADIUS)
        {
            offset.Normalize();
            Vector2D dir(cos(pod.angle *PI/180.0f ),  sin(pod.angle *PI/180.0f ));  
            
            //cerr<< " offset.x " << offset.x << endl;
            //cerr<< " offset.y " << offset.y << endl;
            float angle = abs(acos(offset.DotProduct(dir)));
            if(debug){
                //cerr<< " pod.angle " << pod.angle << " " << angle << " dist " << dist << endl;
                //cerr<< " pthrust " << pthrust << " pthrust*angle*angle " << pthrust*angle*angle<<endl;
            }
            if(pod.passed_checkpoint)
            {
                return dist + pod.speed.Magnitude()*angle*angle * 10.0f;
            } else {
                return dist;
            }
        }
        
        float min_dist = numeric_limits<float>::max();
        int picked_thrust;
        bool passed_checkpoint = pod.passed_checkpoint;
        for(int cangle = 0;  cangle<angles.size(); ++cangle)
        {
            for(int cthrust = 0;  cthrust<thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                //cerr<< " pod.angle " << pod.angle<<endl;
                
                int th  = thrusts[cthrust];
                float dist = p.move(thrusts[cthrust], angles[cangle], false);
                Vector2D t(target);
                if(!chaser){
                    if(p.passed_checkpoint)
                    {
                        t = checkpoints[(pod.check_point_id + 1)%nr_check_points];
                      //  cerr<< "checked 2 " << endl; 
                    } else {
                        t = checkpoints[pod.check_point_id];
                    }
                    
                    float dist_eval =  PickThrustAngleHelper(p, th, t, 
                        angles, thrusts, level-1, chaser, cangle==0 );
                    if(!passed_checkpoint && p.passed_checkpoint)
                    {
                    
                        passed_checkpoint = true;
                        picked_thrust = thrusts[cthrust];
                        min_dist = dist_eval;
                        
                        
                    }else if(min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                    {
                        min_dist = dist_eval;            
                        picked_thrust = thrusts[cthrust];            
                        //if(cangle == 0)
                        //cerr<< " best_pod.angle  " << best_pod.angle << endl; 
                        //cerr<< " best_thrust  " << best_thrust << endl;

                    }
                } else{
                    float dist_eval =  PickThrustAngleHelper(p, th, t, angles, thrusts, level-1, chaser, debug);
                    if(min_dist > dist_eval)
                    {
                        //if(debug)
                        //    cerr<< " dist_eval " << dist_eval<<endl;
                        min_dist = dist_eval;
                        //cerr<< " min_dist " << min_dist<<endl;
                                
                    }
                }

                
                //cerr<< " p.angle " << p.angle << " cthrust " << cthrust << 
                //" dist "<< dist << " dist_eval " << dist_eval<< endl;
               
                                
            }
        }
        if(passed_checkpoint)
        {
            min_dist -=1000000.0f;
        }
        if(level == SIMULATION_LEVEL)
        {
            //pthrust = picked_thrust;
            //cerr<<"pthrust "<< pthrust<<  " picked_thrust " <<picked_thrust<<endl;
        }
        return min_dist;

    }

    //find best target and thrust by simulating real movements 
    void PickThrustAngle(OutputValues& out_pod, const PodData& pod, const int& level)
    {
        //vector<float> angles = { -18.0f * PI/180.0f, -9.0f * PI/180.0f, -2.0f* PI/180.0f, 0.0f, 2.0f* PI/180.0f, 
        //    9.0f*PI/180.0f, 18.0f * PI/180.0f };
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f  };
        vector<float> angles1 = { -18.0f , 0.0f, 18.0f  };
        //vector<float> sinuses = { sin(-18.0f * PI/180.0f), sin(-9.0f * PI/180.0f), sin(-2.0f* PI/180.0f), sin(0.0f), sin(2.0f* PI/180.0f), 
        //    sin(9.0f*PI/180.0f), sin(18.0f * PI/180.0f) };
        //vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        vector<float> thrusts = { 55.0f, 100.0f };
        PodData best_pod;
        float min_dist = numeric_limits<float>::max();
        float best_thrust;
        bool passed_checkpoint = false;
        //cerr<< " pod.angle  " << pod.angle << endl;
        for(int cangle = 0;  cangle<angles.size(); ++cangle)
        {
            for(int cthrust = 0;  cthrust<thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                float dist = p.move(thrusts[cthrust], angles[cangle], cangle == 0);                
                p.passed_checkpoint = p.Collide( checkpoints[p.check_point_id], CHECKPOINT_RADIUS);
                Vector2D target;
                if(p.passed_checkpoint)
                {
                    //cerr<< "checked 1 " << endl; 
                    target = checkpoints[(pod.check_point_id + 1)%nr_check_points];
                    target = getCheckpointPosFromPlayer( pod.position, pod.angle, target);
                } else {
                    target = checkpoints[pod.check_point_id];
                    target = getCheckpointPosFromPlayer( pod.position, pod.angle, target);
                }
                int th = thrusts[cthrust];
                float dist_eval =  PickThrustAngleHelper(p, th, target, angles, thrusts, level, false);
                
                //if(cangle == 0)
                  //  cerr<< " thrusts[cthrust]  " << thrusts[cthrust] << endl;
                if(!passed_checkpoint && p.passed_checkpoint)
                {
                    
                    passed_checkpoint = true;
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th ;//thrusts[cthrust];
                }else if(min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                {
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th;// thrusts[cthrust];
                    //if(cangle == 0)
                    //cerr<< " best_pod.angle  " << best_pod.angle << endl; 
                    //cerr<< " best_thrust  " << best_thrust << endl;

                }                                
            }
        }

        int destX = (int) (pod.position.x + 2000.0f * cos(best_pod.angle * PI/180.0f));
        int destY = (int) (pod.position.y + 2000.0f * sin(best_pod.angle* PI/180.0f));
        Vector2D off =  best_pod.position - pod.position;
        Vector2D out = pod.position + off*100;
        //out_pod.out_x = (int)out.x;     
        //out_pod.out_y = (int)out.y;
        out_pod.out_x = destX;     
        out_pod.out_y = destY;
        cerr<< "best_thrust " << best_thrust<< endl;
        out_pod.sthrust = std::to_string((int)best_thrust);
    }

    //find best target and thrust by simulating real movements 
    //target is the winning opponent
    void PickThrustAngleForChaser(OutputValues& out_pod, const PodData& pod, const Vector2D& target, 
        const int& level)
    {
        //vector<float> angles = { -18.0f * PI/180.0f, -9.0f * PI/180.0f, -2.0f* PI/180.0f, 0.0f, 2.0f* PI/180.0f, 
        //    9.0f*PI/180.0f, 18.0f * PI/180.0f };
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f  };
        //vector<float> sinuses = { sin(-18.0f * PI/180.0f), sin(-9.0f * PI/180.0f), sin(-2.0f* PI/180.0f), sin(0.0f), sin(2.0f* PI/180.0f), 
        //    sin(9.0f*PI/180.0f), sin(18.0f * PI/180.0f) };
        //vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        PodData best_pod;
        float min_dist = numeric_limits<float>::max();
        float best_thrust;
        
        //cerr<< " pod.angle  " << pod.angle << endl;
        for(int cangle = 0;  cangle<angles.size(); ++cangle)
        {
            for(int cthrust = 0;  cthrust<thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                float dist = p.move(thrusts[cthrust], angles[cangle], cangle == 0);                
                //p.passed_checkpoint = p.Collide( checkpoints[p.check_point_id], CHECKPOINT_RADIUS);
                int th = thrusts[cthrust];
                float dist_eval =  PickThrustAngleHelper(p, th, target, 
                    angles, thrusts, level, true);
                
                //if(cangle == 0)
                  //  cerr<< " thrusts[cthrust]  " << thrusts[cthrust] << endl;
                if(min_dist > dist_eval)
                {
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = thrusts[cthrust];
                    //if(cangle == 0)
                    //cerr<< " best_pod.angle  " << best_pod.angle << endl; 
                    //cerr<< " best_thrust  " << best_thrust << endl;

                }
                                
            }
        }
        int destX = (int) (pod.position.x + 2000.0f * cos(best_pod.angle * PI/180.0f));
        int destY = (int) (pod.position.y + 2000.0f * sin(best_pod.angle* PI/180.0f));
        Vector2D off =  best_pod.position - pod.position;
        Vector2D out = pod.position + off*100;
        //out_pod.out_x = (int)out.x;     
        //out_pod.out_y = (int)out.y;
        out_pod.out_x = destX;     
        out_pod.out_y = destY;
        cerr<< "out_pod.out_x " << out_pod.out_x<< endl;
        out_pod.sthrust = std::to_string((int)best_thrust);
    }



    //checks if we should slow dawn when aiming
    void ComputeShouldSlowdawn()
    {
        should_slowdawn.resize(nr_check_points);
        for(int i = 0; i < nr_check_points; ++i )
        {
            Vector2D route1 = getRoute(i);
            Vector2D route2 = getRoute(i+1);
            float angle1 = route1.getAngle();
            float angle2 = route2.getAngle();
            if(abs(angle1 -angle2) <  ((SHOULD_SLOWDAWN_ANGLE_TRESHOLD * PI) / 180.0f))
            {
                should_slowdawn[i] = false;
            } else {
                should_slowdawn[i] = true;
            }

        }
    }

    Game()
    {       
    }

    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int check_point_id, const int a, const int id)
    {   
        pods[id].UpdatePod(pos, s, check_point_id, a);
        cerr << "angle pod " << a <<endl;

    }

    //id of the winning opponents - important for pod  intercepting
    int getWinningOpponentsID()
    {
        int score2 = pods[2].check_point_id + pods[2].lap * nr_check_points;
        int score3 = pods[3].check_point_id + pods[3].lap * nr_check_points;
        //cerr<< "score2 " << score2<< endl;
        //cerr<< "score3 " << score3<< endl;
        //cerr<< "pods[2].lap " << pods[2].lap<< endl;
        //cerr<< "pods[3].lap " << pods[3].lap<< endl;
        //cerr<< "nr_check_points " << nr_check_points<< endl;
        //cerr<< "pods[2].check_point_id " << pods[2].check_point_id<< endl;
        //cerr<< "pods[3].check_point_id " << pods[3].check_point_id<< endl;
        
        if(score2>score3)
        {
            return 2;
        } 
        else if(score2<score3)
        {
            return 3;
        } else {
            const Vector2D& checkpoint_pos =  checkpoints[pods[2].check_point_id];
            Vector2D offset2 = checkpoint_pos - pods[2].position;
            Vector2D offset3 = checkpoint_pos - pods[3].position;
            if(offset2.Magnitude() > offset3.Magnitude())
            {
                return 3;
            } else 
            {
                return 2;
            }
        }

    }

    int getChaserID()
    {
        int score0 = pods[0].check_point_id + pods[0].lap * nr_check_points;
        int score1 = pods[1].check_point_id + pods[1].lap * nr_check_points;
        if(score0>score1)
        {
            return 0;
        } 
        else if(score0<score1)
        {
            return 1;
        } else {
            const Vector2D& checkpoint_pos =  checkpoints[pods[0].check_point_id];
            Vector2D offset0 = checkpoint_pos - pods[0].position;
            Vector2D offset1 = checkpoint_pos - pods[1].position;
            if(offset0.Magnitude() > offset1.Magnitude())
            {
                return 1;
            } else 
            {
                return 0;
            }
        }
    }

    int getInterceptorID()
    {
        return 1 - getChaserID();
    }

    Vector2D getRoute(const int id)
    {
        return (checkpoints[id] - checkpoints[(id-1)%nr_check_points]);
    }

    //find id of the longest route - used to compute boost
    int getLongestRouteID()
    {
        int id = 0;
        float longest_route = std::numeric_limits<float>::min();
        for(int i = 0; i<nr_check_points; ++i)
        {
             Vector2D route = getRoute(i);
             float longeur = getRoute(i).Magnitude();
             cerr<< " longest_route < longeur " << longest_route << " " << longeur << endl;
             cerr<< " route " << route.x << " "<< route.y<<endl; 
             if( longest_route < longeur )
             {
                id =  i;
                longest_route = longeur;
             }
        }
        return id;
    }
    
    void ComputeValues(OutputValues& out_pod1, OutputValues& out_pod2, bool& first )
    {
        int thrust1=MAX_THRUST, thrusts2 = MAX_THRUST;
        if(getChaserID() == 1)
        {
            //swap pods
         //   PodData tmp = pods[0];
           // pods[0] = pods[1];
            //pods[1] = tmp;
        }
        int chaser = 0;// getChaserID();
        int interceptor = 1;// - chaser;// getInterceptorID();
        const Vector2D& checkpoint_pos =  checkpoints[pods[chaser].check_point_id];
        Vector2D offset = checkpoint_pos - pods[chaser].position;
        cerr << "chaser " << chaser << endl;
        
        float checkpoint_dist = offset.Magnitude();

        //checking angle
        /*if (checkpoint_angle > ANGLE_TRESHOLD || checkpoint_angle < -ANGLE_TRESHOLD)
        {
            thrust1 =(int)((float)thrust1 *(1.0f - (float)(checkpoint_angle)));
            thrust1 = std::clamp(thrust1, MIN_THRUST, MAX_THRUST);            
        } */
        //cerr << "thrust1 " << thrust1 << endl;
        
        //checking distance if we need to slow dawn 
        
        
       // cerr << "slow_dawn_treshold " << slow_dawn_treshold << endl;
        //Vector2D prev_route(getRoute(pods[0].check_point_id-1));
        //Vector2D route(getRoute(pods[0].check_point_id));
        //
        //const Vector2D& prev_chposition = checkpoints[(pods[0].check_point_id-1)%nr_check_points];
        /*Vector2D prev_offset = pods[0].position - prev_chposition;
        float prev_dist = prev_offset.Magnitude(); //distance to previous checkpoint
        float route_dist = route.Magnitude();//
        int slow_dawn_treshold = max ( SLOW_DAWN_TRESHOLD, (int)(route_dist/4));*/
        
        /*if( should_slowdawn[pods[0].check_point_id] ){
            if(checkpoint_dist<slow_dawn_treshold)
            {
                float thrust_scale = std::abs((float)checkpoint_dist /(float)slow_dawn_treshold);
                
                thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
                
                
                thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_THRUST)* thrust_scale);
                thrust_scale /= (float)MAX_THRUST;
                thrust1 *= thrust_scale;
                
                cerr << "  thrust_scale " << thrust_scale << endl;
                cerr << "  thrust " << thrust1 << endl;
            } 
            else if(prev_dist <  PREV_SLOW_DAWN_TRESHOLD && route_dist < LONG_ROUTE_TRESHOLD 
            && std::abs(checkpoint_angle) > 90)
            {
                float thrust_scale = std::abs((float)prev_dist / (float) PREV_SLOW_DAWN_TRESHOLD);            
                thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
                thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_PTHRUST)* thrust_scale);
                thrust_scale /= (float)MAX_THRUST;
                
                thrust1 *= thrust_scale;            
                thrust1 = std::clamp(thrust1, MIN_PTHRUST, MAX_THRUST);
                cerr << "  thrust prev dist " << thrust1 << endl;
            }
        }*/
        int opp_id = getWinningOpponentsID();
        if(!first)
        {
            
            PickThrustAngle(out_pod1,  pods[chaser], SIMULATION_LEVEL );
            cerr<< " thrust 11111 " << out_pod1.sthrust << endl;
            //second pod will try to intercept winning opponent
        
        //cerr<< " opp_id "<<opp_id<<endl;
        
            PodData opp(pods[opp_id]);
            for(int i = 0 ; i<OPPONENT_SIMULATION_LEVEL+3; ++i)
            {
                opp.move(100, 0);
            }
            PickThrustAngleForChaser(out_pod2,  pods[interceptor], opp.position , OPPONENT_SIMULATION_LEVEL );
            cerr<< " pods[1].check_point_id " << pods[interceptor].check_point_id << endl;

        } else {
            first = false;
            out_pod1.out_x = checkpoint_pos.x;
            out_pod1.out_y = checkpoint_pos.y;
            out_pod1.sthrust = "BOOST";
            out_pod2.out_x = checkpoint_pos.x;
            out_pod2.out_y = checkpoint_pos.y;
            out_pod2.sthrust = "100";
        }
        Vector2D checkpoint_dir =  checkpoints[pods[chaser].check_point_id];
        checkpoint_dir.Normalize();
        Vector2D opp_pod_dir1(cos(pods[interceptor].angle *PI/180.0f ),  
            sin(pods[interceptor].angle *PI/180.0f ));
        float mspeed = pods[chaser].speed.Magnitude();
        bool collide = pods[chaser].Collide(pods[interceptor].position, 1.2f * POD_RADIUS );
        cerr <<" abs(opp_pod_dir1.DotProduct(checkpoint_dir) " << abs(opp_pod_dir1.DotProduct(checkpoint_dir)) << endl;
        cerr << " pods[1].angle *PI/180.0f " << pods[interceptor].angle *PI/180.0f<<endl;
        if(collide)
            cerr << " collide 1 " <<endl;
        if( collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir1.DotProduct(checkpoint_dir))
         < DOT_TO_SHIELD){
            out_pod1.sthrust = "SHIELD";
        }
        Vector2D opp_pod_dir2(cos(pods[2].angle *PI/180.0f ),  sin(pods[2].angle *PI/180.0f ));
        collide = pods[chaser].Collide(pods[2].position, 1.2f * POD_RADIUS );
        cerr <<" abs(opp_pod_dir2.DotProduct(checkpoint_dir) " << (opp_pod_dir2.DotProduct(checkpoint_dir)) << endl;        
        cerr << " pods[2].angle *PI/180.0f " << pods[2].angle *PI/180.0f<<endl;
        if(collide){
            cerr << " collide 2 " <<endl;
            cerr << " mspeed " << mspeed << endl;
        }
        if( collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir2.DotProduct(checkpoint_dir))
         < DOT_TO_SHIELD){
            out_pod1.sthrust = "SHIELD";
        }
        Vector2D opp_pod_dir3(cos(pods[3].angle *PI/180.0f ),  sin(pods[3].angle *PI/180.0f ));
        cerr << " pods[3].angle *PI/180.0f " << pods[3].angle *PI/180.0f<<endl;
        cerr <<" abs(opp_pod_dir3.DotProduct(checkpoint_dir) " << (opp_pod_dir3.DotProduct(checkpoint_dir)) <<endl;
        collide = pods[chaser].Collide(pods[3].position, 1.2f * POD_RADIUS );
        if(collide)
            cerr << " collide 3 " <<endl;
        if( collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir3.DotProduct(checkpoint_dir))
         < DOT_TO_SHIELD){
            out_pod1.sthrust = "SHIELD";
        }

        cerr<< " thrust 22222 " << out_pod1.sthrust << endl;

        //cerr<< " out_pod1.sthrust " << out_pod1.sthrust << endl;
        //computing boost
        if(has_boost)
        {
            float angle = offset.getAngle();
            float checkpoint_angle  = angle - ((float)pods[chaser].angle * PI/180.0f);
            cerr << " thruuustttt longest_route_id " << longest_route_id << " pods[0].check_point_id "
            <<pods[chaser].check_point_id<<endl;
            cerr << " std::abs(checkpoint_angle) " << std::abs(checkpoint_angle) << " angle "<< angle << endl;
            //use boost on first pod if we are on the longest route betwen 2 checkpoints
            bool use_boost = pods[chaser].check_point_id == longest_route_id && 
                std::abs(checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
            if(use_boost)
            {
                cerr << "boost " << endl;
                out_pod1.sthrust = "BOOST";
                has_boost = false;
            } 
        }
        
                
        //check if we almost collide and 
        bool almost_collide = pods[interceptor].Collide(pods[opp_id].position, 1.2f * POD_RADIUS );
        cerr<< " opp_id "<<opp_id<<endl;
        
        mspeed = pods[interceptor].speed.Magnitude();
        if(almost_collide && mspeed > SPEED_TO_SHIELD)
        {
            out_pod2.sthrust = "SHIELD";
        } //else {
          //  out_pod2.sthrust = "50";
        //}

        
        
    }

    
};


/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

int main()
{
    vector<Vector2D> checkpoints;
    int laps;
    cin >> laps; cin.ignore();
    int checkpoint_count;
    cin >> checkpoint_count; cin.ignore();
    for (int i = 0; i < checkpoint_count; i++) {
        int checkpoint_x;
        int checkpoint_y;
        cin >> checkpoint_x >> checkpoint_y; cin.ignore();
        cerr<< "checkpoint_x  checkpoint_y " << checkpoint_x  <<" "<< checkpoint_y << endl;
        checkpoints.push_back( Vector2D(checkpoint_x, checkpoint_y) );
    }
    Game game(checkpoints, laps);
    bool first = true;

    // game loop
    while (1) {
        for (int i = 0; i < 2; i++) {
            int x; // x position of your pod
            int y; // y position of your pod
            int vx; // x speed of your pod
            int vy; // y speed of your pod
            int angle; // angle of your pod
            int next_check_point_id; // next check point id of your pod
            cin >> x >> y >> vx >> vy >> angle >> next_check_point_id; cin.ignore();
            //cerr << " next_check_point_id " << std::to_string(next_check_point_id) << endl;
            //PodData podData( Vector2D(x, y), Vector2D(vx, vy), angle, next_check_point_id );
            game.UpdatePod(Vector2D(x, y), Vector2D(vx, vy), angle, next_check_point_id, i);
        }
        for (int i = 0; i < 2; i++) {
            int x_2; // x position of the opponent's pod
            int y_2; // y position of the opponent's pod
            int vx_2; // x speed of the opponent's pod
            int vy_2; // y speed of the opponent's pod
            int angle_2; // angle of the opponent's pod
            int next_check_point_id_2; // next check point id of the opponent's pod
            cin >> x_2 >> y_2 >> vx_2 >> vy_2 >> angle_2 >> next_check_point_id_2; cin.ignore();
            //cerr << " next_check_point_id_2 " << std::to_string(next_check_point_id_2) << endl;
            PodData podData( Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2 );
            game.UpdatePod(Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2, i+2 );
        }
        
        OutputValues out_pod1;
        OutputValues out_pod2;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        game.ComputeValues(out_pod1, out_pod2, first);
        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        //cerr << " out_pod1.out_x " << std::to_string(out_pod1.out_x) << endl;
        //cerr << " out_pod1.out_y " << out_pod1.out_y << endl;
        //cerr << " out_pod1.sthrust " << out_pod1.sthrust << endl;
        //cout << "8000 4500 100" << endl;
        //cout << "8000 4500 100" << endl;
        cout << out_pod1.out_x << " " << out_pod1.out_y << " " << out_pod1.sthrust << endl;
        cout << out_pod2.out_x << " " << out_pod2.out_y << " " << out_pod2.sthrust << endl;
        
    }
}


        PodData best_pod;
        float min_dist = numeric_limits<float>::max();
        float best_thrust;
        bool passed_checkpoint = false;
        //cerr<< " pod.angle  " << pod.angle << endl;
        for(int cangle = 0;  cangle<angles.size(); ++cangle)
        {
            for(int cthrust = 0;  cthrust<thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                float dist = p.move(thrusts[cthrust], angles[cangle], cangle == 0);                
                p.passed_checkpoint = p.Collide( checkpoints[p.check_point_id], CHECKPOINT_RADIUS);
                Vector2D target;
                if(p.passed_checkpoint)
                {
                    //cerr<< "checked 1 " << endl; 
                    target = checkpoints[(pod.check_point_id + 1)%nr_check_points];
                } else {
                    target = checkpoints[pod.check_point_id];
                }
                int th = thrusts[cthrust];
                float dist_eval =  PickThrustAngleHelper(p, th, target, angles, thrusts, level, false);
                
                //if(cangle == 0)
                  //  cerr<< " thrusts[cthrust]  " << thrusts[cthrust] << endl;
                if(!passed_checkpoint && p.passed_checkpoint)
                {
                    
                    passed_checkpoint = true;
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th ;//thrusts[cthrust];
                }else if(min_dist > dist_eval && passed_checkpoint == p.passed_checkpoint)
                {
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = th;// thrusts[cthrust];
                    //if(cangle == 0)
                    //cerr<< " best_pod.angle  " << best_pod.angle << endl; 
                    //cerr<< " best_thrust  " << best_thrust << endl;

                }                                
            }
        }

        int destX = (int) (pod.position.x + 2000.0f * cos(best_pod.angle * PI/180.0f));
        int destY = (int) (pod.position.y + 2000.0f * sin(best_pod.angle* PI/180.0f));
        Vector2D off =  best_pod.position - pod.position;
        Vector2D out = pod.position + off*100;
        //out_pod.out_x = (int)out.x;     
        //out_pod.out_y = (int)out.y;
        out_pod.out_x = destX;     
        out_pod.out_y = destY;
        cerr<< "best_thrust " << best_thrust<< endl;
        out_pod.sthrust = std::to_string((int)best_thrust);
    }

    //find best target and thrust by simulating real movements 
    //target is the winning opponent
    void PickThrustAngleForChaser(OutputValues& out_pod, const PodData& pod, const Vector2D& target, 
        const int& level)
    {
        //vector<float> angles = { -18.0f * PI/180.0f, -9.0f * PI/180.0f, -2.0f* PI/180.0f, 0.0f, 2.0f* PI/180.0f, 
        //    9.0f*PI/180.0f, 18.0f * PI/180.0f };
        vector<float> angles = { -18.0f ,-9.0f, 0.0f, 9.0f, 18.0f  };
        //vector<float> sinuses = { sin(-18.0f * PI/180.0f), sin(-9.0f * PI/180.0f), sin(-2.0f* PI/180.0f), sin(0.0f), sin(2.0f* PI/180.0f), 
        //    sin(9.0f*PI/180.0f), sin(18.0f * PI/180.0f) };
        //vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        vector<float> thrusts = { 5.0f, 50.0f, 100.0f };
        PodData best_pod;
        float min_dist = numeric_limits<float>::max();
        float best_thrust;
        
        //cerr<< " pod.angle  " << pod.angle << endl;
        for(int cangle = 0;  cangle<angles.size(); ++cangle)
        {
            for(int cthrust = 0;  cthrust<thrusts.size(); ++cthrust)
            {
                PodData p(pod);
                float dist = p.move(thrusts[cthrust], angles[cangle], cangle == 0);                
                //p.passed_checkpoint = p.Collide( checkpoints[p.check_point_id], CHECKPOINT_RADIUS);
                int th = thrusts[cthrust];
                float dist_eval =  PickThrustAngleHelper(p, th, target, 
                    angles, thrusts, level, true);
                
                //if(cangle == 0)
                  //  cerr<< " thrusts[cthrust]  " << thrusts[cthrust] << endl;
                if(min_dist > dist_eval)
                {
                    min_dist = dist_eval;
                    best_pod = p;
                    best_thrust = thrusts[cthrust];
                    //if(cangle == 0)
                    //cerr<< " best_pod.angle  " << best_pod.angle << endl; 
                    //cerr<< " best_thrust  " << best_thrust << endl;

                }
                                
            }
        }
        int destX = (int) (pod.position.x + 2000.0f * cos(best_pod.angle * PI/180.0f));
        int destY = (int) (pod.position.y + 2000.0f * sin(best_pod.angle* PI/180.0f));
        Vector2D off =  best_pod.position - pod.position;
        Vector2D out = pod.position + off*100;
        //out_pod.out_x = (int)out.x;     
        //out_pod.out_y = (int)out.y;
        out_pod.out_x = destX;     
        out_pod.out_y = destY;
        cerr<< "out_pod.out_x " << out_pod.out_x<< endl;
        out_pod.sthrust = std::to_string((int)best_thrust);
    }



    //checks if we should slow dawn when aiming
    void ComputeShouldSlowdawn()
    {
        should_slowdawn.resize(nr_check_points);
        for(int i = 0; i < nr_check_points; ++i )
        {
            Vector2D route1 = getRoute(i);
            Vector2D route2 = getRoute(i+1);
            float angle1 = route1.getAngle();
            float angle2 = route2.getAngle();
            if(abs(angle1 -angle2) <  ((SHOULD_SLOWDAWN_ANGLE_TRESHOLD * PI) / 180.0f))
            {
                should_slowdawn[i] = false;
            } else {
                should_slowdawn[i] = true;
            }

        }
    }

    Game()
    {       
    }

    void UpdatePod(const Vector2D& pos, const Vector2D& s, const int check_point_id, const int a, const int id)
    {   
        pods[id].UpdatePod(pos, s, check_point_id, a);
        cerr << "angle pod " << a <<endl;

    }

    //id of the winning opponents - important for pod  intercepting
    int getWinningOpponentsID()
    {
        int score2 = pods[2].check_point_id + pods[2].lap * nr_check_points;
        int score3 = pods[3].check_point_id + pods[3].lap * nr_check_points;
        //cerr<< "score2 " << score2<< endl;
        //cerr<< "score3 " << score3<< endl;
        //cerr<< "pods[2].lap " << pods[2].lap<< endl;
        //cerr<< "pods[3].lap " << pods[3].lap<< endl;
        //cerr<< "nr_check_points " << nr_check_points<< endl;
        //cerr<< "pods[2].check_point_id " << pods[2].check_point_id<< endl;
        //cerr<< "pods[3].check_point_id " << pods[3].check_point_id<< endl;
        
        if(score2>score3)
        {
            return 2;
        } 
        else if(score2<score3)
        {
            return 3;
        } else {
            const Vector2D& checkpoint_pos =  checkpoints[pods[2].check_point_id];
            Vector2D offset2 = checkpoint_pos - pods[2].position;
            Vector2D offset3 = checkpoint_pos - pods[3].position;
            if(offset2.Magnitude() > offset3.Magnitude())
            {
                return 3;
            } else 
            {
                return 2;
            }
        }

    }

    int getChaserID()
    {
        int score0 = pods[0].check_point_id + pods[0].lap * nr_check_points;
        int score1 = pods[1].check_point_id + pods[1].lap * nr_check_points;
        if(score0>score1)
        {
            return 0;
        } 
        else if(score0<score1)
        {
            return 1;
        } else {
            const Vector2D& checkpoint_pos =  checkpoints[pods[0].check_point_id];
            Vector2D offset0 = checkpoint_pos - pods[0].position;
            Vector2D offset1 = checkpoint_pos - pods[1].position;
            if(offset0.Magnitude() > offset1.Magnitude())
            {
                return 1;
            } else 
            {
                return 0;
            }
        }
    }

    int getInterceptorID()
    {
        return 1 - getChaserID();
    }

    Vector2D getRoute(const int id)
    {
        return (checkpoints[id] - checkpoints[(id-1)%nr_check_points]);
    }

    //find id of the longest route - used to compute boost
    int getLongestRouteID()
    {
        int id = 0;
        float longest_route = std::numeric_limits<float>::min();
        for(int i = 0; i<nr_check_points; ++i)
        {
             Vector2D route = getRoute(i);
             float longeur = getRoute(i).Magnitude();
             cerr<< " longest_route < longeur " << longest_route << " " << longeur << endl;
             cerr<< " route " << route.x << " "<< route.y<<endl; 
             if( longest_route < longeur )
             {
                id =  i;
                longest_route = longeur;
             }
        }
        return id;
    }
    
    void ComputeValues(OutputValues& out_pod1, OutputValues& out_pod2, bool& first )
    {
        int thrust1=MAX_THRUST, thrusts2 = MAX_THRUST;
        if(getChaserID() == 1)
        {
            //swap pods
         //   PodData tmp = pods[0];
           // pods[0] = pods[1];
            //pods[1] = tmp;
        }
        int chaser = 0;// getChaserID();
        int interceptor = 1;// - chaser;// getInterceptorID();
        const Vector2D& checkpoint_pos =  checkpoints[pods[chaser].check_point_id];
        Vector2D offset = checkpoint_pos - pods[chaser].position;
        cerr << "chaser " << chaser << endl;
        
        float checkpoint_dist = offset.Magnitude();

        //checking angle
        /*if (checkpoint_angle > ANGLE_TRESHOLD || checkpoint_angle < -ANGLE_TRESHOLD)
        {
            thrust1 =(int)((float)thrust1 *(1.0f - (float)(checkpoint_angle)));
            thrust1 = std::clamp(thrust1, MIN_THRUST, MAX_THRUST);            
        } */
        //cerr << "thrust1 " << thrust1 << endl;
        
        //checking distance if we need to slow dawn 
        
        
       // cerr << "slow_dawn_treshold " << slow_dawn_treshold << endl;
        //Vector2D prev_route(getRoute(pods[0].check_point_id-1));
        //Vector2D route(getRoute(pods[0].check_point_id));
        //
        //const Vector2D& prev_chposition = checkpoints[(pods[0].check_point_id-1)%nr_check_points];
        /*Vector2D prev_offset = pods[0].position - prev_chposition;
        float prev_dist = prev_offset.Magnitude(); //distance to previous checkpoint
        float route_dist = route.Magnitude();//
        int slow_dawn_treshold = max ( SLOW_DAWN_TRESHOLD, (int)(route_dist/4));*/
        
        /*if( should_slowdawn[pods[0].check_point_id] ){
            if(checkpoint_dist<slow_dawn_treshold)
            {
                float thrust_scale = std::abs((float)checkpoint_dist /(float)slow_dawn_treshold);
                
                thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
                
                
                thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_THRUST)* thrust_scale);
                thrust_scale /= (float)MAX_THRUST;
                thrust1 *= thrust_scale;
                
                cerr << "  thrust_scale " << thrust_scale << endl;
                cerr << "  thrust " << thrust1 << endl;
            } 
            else if(prev_dist <  PREV_SLOW_DAWN_TRESHOLD && route_dist < LONG_ROUTE_TRESHOLD 
            && std::abs(checkpoint_angle) > 90)
            {
                float thrust_scale = std::abs((float)prev_dist / (float) PREV_SLOW_DAWN_TRESHOLD);            
                thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
                thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_PTHRUST)* thrust_scale);
                thrust_scale /= (float)MAX_THRUST;
                
                thrust1 *= thrust_scale;            
                thrust1 = std::clamp(thrust1, MIN_PTHRUST, MAX_THRUST);
                cerr << "  thrust prev dist " << thrust1 << endl;
            }
        }*/
        int opp_id = getWinningOpponentsID();
        if(!first)
        {
            
            PickThrustAngle(out_pod1,  pods[chaser], SIMULATION_LEVEL );
            cerr<< " thrust 11111 " << out_pod1.sthrust << endl;
            //second pod will try to intercept winning opponent
        
        //cerr<< " opp_id "<<opp_id<<endl;
        
            PodData opp(pods[opp_id]);
            for(int i = 0 ; i<OPPONENT_SIMULATION_LEVEL+3; ++i)
            {
                opp.move(100, 0);
            }
            PickThrustAngleForChaser(out_pod2,  pods[interceptor], opp.position , OPPONENT_SIMULATION_LEVEL );
            cerr<< " pods[1].check_point_id " << pods[interceptor].check_point_id << endl;

        } else {
            first = false;
            out_pod1.out_x = checkpoint_pos.x;
            out_pod1.out_y = checkpoint_pos.y;
            out_pod1.sthrust = "BOOST";
            out_pod2.out_x = checkpoint_pos.x;
            out_pod2.out_y = checkpoint_pos.y;
            out_pod2.sthrust = "100";
        }
        Vector2D checkpoint_dir =  checkpoints[pods[chaser].check_point_id];
        checkpoint_dir.Normalize();
        Vector2D opp_pod_dir1(cos(pods[interceptor].angle *PI/180.0f ),  
            sin(pods[interceptor].angle *PI/180.0f ));
        float mspeed = pods[chaser].speed.Magnitude();
        bool collide = pods[chaser].Collide(pods[interceptor].position, 1.2f * POD_RADIUS );
        cerr <<" abs(opp_pod_dir1.DotProduct(checkpoint_dir) " << abs(opp_pod_dir1.DotProduct(checkpoint_dir)) << endl;
        cerr << " pods[1].angle *PI/180.0f " << pods[interceptor].angle *PI/180.0f<<endl;
        if(collide)
            cerr << " collide 1 " <<endl;
        if( collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir1.DotProduct(checkpoint_dir))
         < DOT_TO_SHIELD){
            out_pod1.sthrust = "SHIELD";
        }
        Vector2D opp_pod_dir2(cos(pods[2].angle *PI/180.0f ),  sin(pods[2].angle *PI/180.0f ));
        collide = pods[chaser].Collide(pods[2].position, 1.2f * POD_RADIUS );
        cerr <<" abs(opp_pod_dir2.DotProduct(checkpoint_dir) " << (opp_pod_dir2.DotProduct(checkpoint_dir)) << endl;        
        cerr << " pods[2].angle *PI/180.0f " << pods[2].angle *PI/180.0f<<endl;
        if(collide){
            cerr << " collide 2 " <<endl;
            cerr << " mspeed " << mspeed << endl;
        }
        if( collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir2.DotProduct(checkpoint_dir))
         < DOT_TO_SHIELD){
            out_pod1.sthrust = "SHIELD";
        }
        Vector2D opp_pod_dir3(cos(pods[3].angle *PI/180.0f ),  sin(pods[3].angle *PI/180.0f ));
        cerr << " pods[3].angle *PI/180.0f " << pods[3].angle *PI/180.0f<<endl;
        cerr <<" abs(opp_pod_dir3.DotProduct(checkpoint_dir) " << (opp_pod_dir3.DotProduct(checkpoint_dir)) <<endl;
        collide = pods[chaser].Collide(pods[3].position, 1.2f * POD_RADIUS );
        if(collide)
            cerr << " collide 3 " <<endl;
        if( collide && mspeed > SPEED_TO_SHIELD && (opp_pod_dir3.DotProduct(checkpoint_dir))
         < DOT_TO_SHIELD){
            out_pod1.sthrust = "SHIELD";
        }

        cerr<< " thrust 22222 " << out_pod1.sthrust << endl;

        //cerr<< " out_pod1.sthrust " << out_pod1.sthrust << endl;
        //computing boost
        if(has_boost)
        {
            float angle = offset.getAngle();
            float checkpoint_angle  = angle - ((float)pods[chaser].angle * PI/180.0f);
            cerr << " thruuustttt longest_route_id " << longest_route_id << " pods[0].check_point_id "
            <<pods[chaser].check_point_id<<endl;
            cerr << " std::abs(checkpoint_angle) " << std::abs(checkpoint_angle) << " angle "<< angle << endl;
            //use boost on first pod if we are on the longest route betwen 2 checkpoints
            bool use_boost = pods[chaser].check_point_id == longest_route_id && 
                std::abs(checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
            if(use_boost)
            {
                cerr << "boost " << endl;
                out_pod1.sthrust = "BOOST";
                has_boost = false;
            } 
        }
        
                
        //check if we almost collide and 
        bool almost_collide = pods[interceptor].Collide(pods[opp_id].position, 1.2f * POD_RADIUS );
        cerr<< " opp_id "<<opp_id<<endl;
        
        mspeed = pods[interceptor].speed.Magnitude();
        if(almost_collide && mspeed > SPEED_TO_SHIELD)
        {
            out_pod2.sthrust = "SHIELD";
        } //else {
          //  out_pod2.sthrust = "50";
        //}

        
        
    }

    
};


/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

int main()
{
    vector<Vector2D> checkpoints;
    int laps;
    cin >> laps; cin.ignore();
    int checkpoint_count;
    cin >> checkpoint_count; cin.ignore();
    for (int i = 0; i < checkpoint_count; i++) {
        int checkpoint_x;
        int checkpoint_y;
        cin >> checkpoint_x >> checkpoint_y; cin.ignore();
        cerr<< "checkpoint_x  checkpoint_y " << checkpoint_x  <<" "<< checkpoint_y << endl;
        checkpoints.push_back( Vector2D(checkpoint_x, checkpoint_y) );
    }
    Game game(checkpoints, laps);
    bool first = true;

    // game loop
    while (1) {
        for (int i = 0; i < 2; i++) {
            int x; // x position of your pod
            int y; // y position of your pod
            int vx; // x speed of your pod
            int vy; // y speed of your pod
            int angle; // angle of your pod
            int next_check_point_id; // next check point id of your pod
            cin >> x >> y >> vx >> vy >> angle >> next_check_point_id; cin.ignore();
            //cerr << " next_check_point_id " << std::to_string(next_check_point_id) << endl;
            //PodData podData( Vector2D(x, y), Vector2D(vx, vy), angle, next_check_point_id );
            game.UpdatePod(Vector2D(x, y), Vector2D(vx, vy), angle, next_check_point_id, i);
        }
        for (int i = 0; i < 2; i++) {
            int x_2; // x position of the opponent's pod
            int y_2; // y position of the opponent's pod
            int vx_2; // x speed of the opponent's pod
            int vy_2; // y speed of the opponent's pod
            int angle_2; // angle of the opponent's pod
            int next_check_point_id_2; // next check point id of the opponent's pod
            cin >> x_2 >> y_2 >> vx_2 >> vy_2 >> angle_2 >> next_check_point_id_2; cin.ignore();
            //cerr << " next_check_point_id_2 " << std::to_string(next_check_point_id_2) << endl;
            PodData podData( Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2 );
            game.UpdatePod(Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2, i+2 );
        }
        
        OutputValues out_pod1;
        OutputValues out_pod2;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        game.ComputeValues(out_pod1, out_pod2, first);
        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        //cerr << " out_pod1.out_x " << std::to_string(out_pod1.out_x) << endl;
        //cerr << " out_pod1.out_y " << out_pod1.out_y << endl;
        //cerr << " out_pod1.sthrust " << out_pod1.sthrust << endl;
        //cout << "8000 4500 100" << endl;
        //cout << "8000 4500 100" << endl;
        cout << out_pod1.out_x << " " << out_pod1.out_y << " " << out_pod1.sthrust << endl;
        cout << out_pod2.out_x << " " << out_pod2.out_y << " " << out_pod2.sthrust << endl;
        
    }
}

