
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
    constexpr int BOOST_ANGLE_TRESHOLD = 5;
    constexpr int BOOST_DISTANCE_TRESHOLD = 4000;
    constexpr int LONG_ROUTE_TRESHOLD = 4000;
    
    constexpr float ANGLE_TRESHOLD = 80.0f/90.0f;
    constexpr int MAX_THRUST = 100;
    constexpr int MIN_THRUST = 60;
    constexpr int MIN_PTHRUST = 30;
    constexpr int PREV_SLOW_DAWN_TRESHOLD = 1500;
    constexpr int COLLISION_TRESHOLD = 2500;
    constexpr int PLAYER_COLLISION_TRESHOLD = 2100;
    constexpr int CHECKPOINT_RADIUS = 600;
    constexpr int POD_RADIUS = 400;

    constexpr int COLLISIONS_ANGLE_TRESHOLD = 15;
    constexpr float PI = 3.14f;
    constexpr float SHOULD_SLOWDAWN_ANGLE_TRESHOLD = 60.0f;
    
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
        PodData(const Vector2D& pos, const Vector2D& s, const int id, const int a ) :
            position(pos), speed(s), check_point_id(id), angle(a), lap(0)
        {            
        }
        PodData(){};
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
            target_checkpoint = -1;
        }

        setTargetCheckpoint()
        bool Collide(const Vector2D& pos, const float collision_radius)
        {
            Vector2D offset = position-pos;
            cerr<< " offset.Magnitude() "<< offset.Magnitude() << endl;
            return  offset.Magnitude() < collision_radius * 2.0f;
        }
        
    public:
        Vector2D position;
        Vector2D speed;
        int check_point_id;
        int angle;
        int lap;
        int target_checkpoint;
        
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
             float longeur = route.Magnitude();
             if( longest_route < longeur )
             {
                id = i;// (i-1)%nr_check_points;
                longest_route = longeur;
             }
        }
        return id;
    }
    
    void ComputeValues(OutputValues& out_pod1, OutputValues& out_pod2 )
    {
        int thrust1=MAX_THRUST, thrusts2 = MAX_THRUST;
        const Vector2D& checkpoint_pos =  checkpoints[pods[0].check_point_id];
        Vector2D offset = checkpoint_pos - pods[0].position;
        float angle = offset.getAngle();
        float checkpoint_angle  = angle - pods[0].speed.getAngle();
        //checking angle
        if (checkpoint_angle > ANGLE_TRESHOLD || checkpoint_angle < -ANGLE_TRESHOLD)
        {
            thrust1 =(int)((float)thrust1 *(1.0f - (float)(checkpoint_angle)));
            thrust1 = std::clamp(thrust1, MIN_THRUST, MAX_THRUST);            
        } 
        cerr << "thrust1 " << thrust1 << endl;
        
        //checking distance if we need to slow dawn 
        float checkpoint_dist = offset.Magnitude();
        
       // cerr << "slow_dawn_treshold " << slow_dawn_treshold << endl;
        //Vector2D prev_route(getRoute(pods[0].check_point_id-1));
        Vector2D route(getRoute(pods[0].check_point_id));
        //
        const Vector2D& prev_chposition = checkpoints[(pods[0].check_point_id-1)%nr_check_points];
        Vector2D prev_offset = pods[0].position - prev_chposition;
        float prev_dist = prev_offset.Magnitude(); //distance to previous checkpoint
        float route_dist = route.Magnitude();//
        int slow_dawn_treshold = max ( SLOW_DAWN_TRESHOLD, (int)(route_dist/4));
        if( should_slowdawn[pods[0].check_point_id] ){
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
        }
        out_pod1.out_x = checkpoint_pos.x;
        out_pod1.out_y = checkpoint_pos.y;
        
        //computing boost
        if(has_boost)
        {
            //use boost on first pod if we are on the longest route betwen 2 checkpoints
            bool use_boost = pods[0].check_point_id == longest_route_id && 
                std::abs(checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
            if(use_boost)
            {
                cerr << "boost " << endl;
                out_pod1.sthrust = "BOOST";
                has_boost = false;
            } else{
                cerr << " thruuustttt " << endl;
                out_pod1.sthrust = std::to_string(thrust1);
            }
        } else {
            out_pod1.sthrust = std::to_string(thrust1);
        }
        
        //second pod will try to intercept winning opponent
        int opp_id = getWinningOpponentsID();
        //cerr<< " opp_id "<<opp_id<<endl;
        Vector2D speed_offset = pods[opp_id].speed * 3.0f;

        
        //check if we almost collide and 
        bool almost_collide = pods[1].Collide(pods[opp_id].position, 1.3f*(float)POD_RADIUS );
        cerr<< " opp_id "<<opp_id<<endl;
        if(almost_collide ){
            out_pod2.out_x = pods[opp_id].position.x ;//+ speed_offset.x;
            out_pod2.out_y = pods[opp_id].position.y ;//+ speed_offset.y;
        } else {
            out_pod2.out_x = checkpoints[nr_check_points-1].x; 
            out_pod2.out_y = checkpoints[nr_check_points-1].y;
        }
        float angle_op = pods[opp_id].speed.getAngle();
        float angle_pod = pods[1].speed.getAngle();
        cerr<< " angle_op "<<angle_op<<endl;
        cerr<< " angle_pod "<<angle_pod<<endl;
        if(almost_collide )//&& std::abs(angle_op - angle_pod)> (180.0f / PI)* (float)COLLISIONS_ANGLE_TRESHOLD )
        {
            out_pod2.sthrust = "SHIELD";
        } else {
            out_pod2.sthrust = "50";
        }

        
        
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
            cerr << " next_check_point_id " << std::to_string(next_check_point_id) << endl;
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
            cerr << " next_check_point_id_2 " << std::to_string(next_check_point_id_2) << endl;
            PodData podData( Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2 );
            game.UpdatePod(Vector2D(x_2, y_2), Vector2D(vx_2, vy_2), angle_2, next_check_point_id_2, i+2 );
        }
        
        OutputValues out_pod1;
        OutputValues out_pod2;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        //cout << out_pod1.out_x << " " << out_pod1.out_y << " " << "100" << endl;
        game.ComputeValues(out_pod1, out_pod2);
        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        cerr << " out_pod1.out_x " << std::to_string(out_pod1.out_x) << endl;
        cerr << " out_pod1.out_y " << out_pod1.out_y << endl;
        cerr << " out_pod1.sthrust " << out_pod1.sthrust << endl;
        //cout << "8000 4500 100" << endl;
        //cout << "8000 4500 100" << endl;
        cout << out_pod1.out_x << " " << out_pod1.out_y << " " << out_pod1.sthrust << endl;
        cout << out_pod2.out_x << " " << out_pod2.out_y << " " << out_pod2.sthrust << endl;
        
    }
}

