#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

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
    
    constexpr int ANGLE_TRESHOLD = 80;
    constexpr int MAX_THRUST = 100;
    constexpr int MIN_THRUST = 60;
    constexpr int MIN_PTHRUST = 30;
    constexpr int PREV_SLOW_DAWN_TRESHOLD = 1500;
    constexpr int COLLISION_TRESHOLD = 2500;
    constexpr int PLAYER_COLLISION_TRESHOLD = 2100;
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


class Game
{
    private:
        Vector2D in_pos;
        
        Vector2D checkpoint;
        
        Vector2D prev_checkpoint;
        
        int checkpoint_dist;
        int checkpoint_angle;
        Vector2D opponent_pos;
        float prev_dist;
        float route_dist;
        Vector2D speed;
        
    public:
    Game()
    {
        in_pos = Vector2D::Zero();
        checkpoint = Vector2D::Zero();
        
        checkpoint_dist = 0;
        checkpoint_angle = 0;
        opponent_pos = Vector2D::Zero();
        
        prev_checkpoint = Vector2D(1000000000, 1000000000);
        
        route_dist = std::numeric_limits<float>::max();        
    }

    void UpdateParams(const int& x,
                const int& y,
                const int& next_checkpoint_x,
                const int& next_checkpoint_y,
                const int& next_checkpoint_dist,
                const int& next_checkpoint_angle,        
                const int& opp_x,
                const int& opp_y)
    {
        speed = Vector2D(x,y) - in_pos;
        
        in_pos = Vector2D(x,y);
        
       // bool changepod = false;
        Vector2D next_checkpoint(next_checkpoint_x, next_checkpoint_y);
        if(checkpoint != next_checkpoint )
        {
            prev_checkpoint = checkpoint;
            route_dist = next_checkpoint_dist;
            //changepod = true;
        }

        Vector2D offset = in_pos - prev_checkpoint;
        
        prev_dist  = offset.Magnitude();
        
        
        checkpoint = Vector2D(next_checkpoint_x,next_checkpoint_y) ;
        checkpoint -= speed*3.0f;
        
        checkpoint_dist = next_checkpoint_dist;
        checkpoint_angle = next_checkpoint_angle;
        opponent_pos = Vector2D(opp_x, opp_y);
        
    }

    void ComputeValues(int& out_x, int& out_y, std::string& sthrust, bool& boost)
    {
        int thrust;
        //checking angle
        if (checkpoint_angle > ANGLE_TRESHOLD || checkpoint_angle < -ANGLE_TRESHOLD)
        {
            thrust = (int)(1.0f - (float)(checkpoint_angle)/90.0f);
            thrust = std::clamp(thrust, MIN_THRUST, MAX_THRUST);            
        }            
        else{             
            thrust = MAX_THRUST;
        }

        cerr << "distance " << checkpoint_dist << endl;
        cerr << "angle " << checkpoint_angle << endl;
        cerr << "route_dist " << route_dist << endl;
        
        
        //checking distance
        
        cerr << " prev_dist " << prev_dist << endl;
        int slow_dawn_treshold = max ( SLOW_DAWN_TRESHOLD, (int)(route_dist/4));
        cerr << "slow_dawn_treshold " << slow_dawn_treshold << endl;
        if(checkpoint_dist<slow_dawn_treshold)
        {
            float thrust_scale = std::abs((float)checkpoint_dist /(float)slow_dawn_treshold);
            
            thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
            
            
            thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_THRUST)* thrust_scale);
            thrust_scale /= (float)MAX_THRUST;
            thrust *= thrust_scale;
            
            cerr << "  thrust_scale " << thrust_scale << endl;
            cerr << "  thrust " << thrust << endl;
        }//checking if we should slow dawn after passing checkpoint 
        else if(prev_dist <  PREV_SLOW_DAWN_TRESHOLD && route_dist < LONG_ROUTE_TRESHOLD 
        && std::abs(checkpoint_angle) > 90)
        {
            float thrust_scale = std::abs((float)prev_dist / (float) PREV_SLOW_DAWN_TRESHOLD);            
            thrust_scale = std::clamp(thrust_scale, 0.0f, 1.0f);
            thrust_scale = (float)MIN_THRUST + ((float)(MAX_THRUST-MIN_THRUST)* thrust_scale);
            thrust_scale /= (float)MAX_THRUST;
            
            thrust *= thrust_scale;            
            thrust = std::clamp(thrust, MIN_PTHRUST, MAX_THRUST);
            cerr << "  thrust prev dist " << thrust << endl;
        }
        
        Vector2D offset_pl_opp =  in_pos - opponent_pos;
        
        float opponent_player_dist = offset_pl_opp.Magnitude();
        Vector2D offset_pod_opp =  checkpoint - opponent_pos;
        
        float opponent_pod_dist = offset_pod_opp.Magnitude();
        cerr << " opponent_player_dist " <<  opponent_player_dist<< endl;
        if(PLAYER_COLLISION_TRESHOLD > opponent_player_dist && std::abs(checkpoint_angle) < 90 )
        {
            thrust = MAX_THRUST;
        }
       // if(COLLISION_TRESHOLD > opponent_pod_dist && COLLISION_TRESHOLD >  checkpoint_dist)
        {
         //   thrust = 70;
        }
        //computing boost
        if(!boost)
        {
            boost =  std::abs(checkpoint_angle)  < BOOST_ANGLE_TRESHOLD && checkpoint_dist > BOOST_DISTANCE_TRESHOLD;
            if(boost)
            {
                cerr << "boost " << endl;
                sthrust = "BOOST";
            } else{
                cerr << " thruuustttt " << endl;
                sthrust = std::to_string(thrust);
            }
        } else {
            sthrust = std::to_string(thrust);
        }
        
        out_x = checkpoint.x;
        out_y = checkpoint.y;
        
    }

    
};

int main()
{
    bool boost = false;
    Game game;
    // game loop
    while (1) {
        int x;
        int y;
        int next_checkpoint_x; // x position of the next check point
        int next_checkpoint_y; // y position of the next check point
        int next_checkpoint_dist; // distance to the next checkpoint
        int next_checkpoint_angle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> next_checkpoint_x >> next_checkpoint_y >> next_checkpoint_dist >> next_checkpoint_angle; cin.ignore();
        int opponent_x;
        int opponent_y;
        cin >> opponent_x >> opponent_y; cin.ignore();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
 
        game.UpdateParams(x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle, opponent_x, opponent_y);

        int pos_x, pos_y;
        std::string thrust;                        
        game.ComputeValues(pos_x, pos_y, thrust, boost);
        cout << pos_x << " " << pos_y << " " << thrust << endl;        
        
    }
}