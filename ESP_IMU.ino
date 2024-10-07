#include "ServoDriver.h"
#include "Utils.h"
#include "Ultrasonic.h"
#include "Gait.h"
#include "IMU.h"


#define FIRMWARE_VERSION 21

#define FLAG_DEBUG  1
#define FLAG_PAUSED 2
#define FLAG_POWER  4
#define FLAG_BARK   8
#define FLAG_IMU    16

#define BARK_PLAY_E 12

#define A0 13
#define A1 14

#define OEPIN 0

struct Flags
{
  unsigned char flags;
  bool isset( unsigned char flag )
  {
    return (flags & flag);
  }
  void set( unsigned char flag )
  {
    flags |= flag;
  }
  void unset( unsigned char flag )
  {
    flags &= ~flag;
  }
};
Flags flags = { FLAG_PAUSED };


ServoDriver servo_driver;
void servogoto( char servo_id, float pos )
{
 // Serial.println(String((int) servo_id) + ": " + String(pos));
  servo_driver.gotoPos( servo_id, ( pos + 90.0 ) );
}
void println( const char* name, float val )
{
  Serial.print( name );
  Serial.print( ": " );
  Serial.println( val );
}


//mm
#define ROTRADIUS 31
#define LTHIGH 100.0
#define LCALF  125.327

#define ROTATOR 0
#define HIP     1
#define KNEE    2


#define  FL    0
#define  BL    1
#define  FR    2
#define  BR    3
#define  FRONT 4
#define  BACK  5
#define  LEFT  6
#define  RIGHT 7
#define  ALL   8

#define REVERSED 1
#define IS_XYZ   2

struct LegInfo {
  char ids[3]; //servo_ids of each. todo: cant we calculate these?
  signed char init_pos[3]; //dont go outside -127 or +127

  //how much offset to apply to make centered at zero. this is for installation offset, not runtime
  signed char offset_pos[3];
  //bool reversed;
  //bool is_xyz;
  Flags flags;
};

#define NUM_LEGS 4
//fl, bl, fr, br
LegInfo leg_info[] = {
  { {2, 1, 0}, {0, 0, 0}, { 27, -30, -30}, 0 }, //false, false },  //change to 0,0,-45 for defaults
  { {5, 4, 3}, {0, 0, 0}, { -12, 5, -45}, 0 }, //false, false  },  //change to 0,0,-45
  { {8, 7, 6}, {0, 0, 0}, { 20, 5, 45}, REVERSED }, //true,  false },  //change to 0,0,45
  { {11, 10, 9}, {0, 0, 0}, { 25, -5, 40}, REVERSED } //true,  false  },  //change to 0,0,45
};


signed char offsets[4][3] = {0};
signed char gait_offsets[4][3] = {0};


#define MOUTH_OPEN_MAX 45
#define MOUTH_CLOSE     0
#define MOUTH_OPEN     20
signed char mouth_pos = MOUTH_CLOSE;
signed char mouth_target = MOUTH_CLOSE;
//char mouth_move_type = 0; //0 normal, 1 bark

GaitLegParam gait_trot = {
  //amplitude, center, phase, clipmin%, clipmax%
  { {30,  0,  0, -100, 100 }, //x
    {20, 190, 90, -100,  20 }, //y
    {0,   0,  0, -100, 100 }  //z
  },

  //if foot down and up phases were of equal length
  //0 deg is at bottom of middle of down phase
  //at x=0 and y=max. at y=max, foot is most below hip
  //so touchdown start happens at -90, or 270
  //and end happens at +90
  290, //touchdown_start_deg //270 + 20
  70, //touchdown_end_deg    //90 - 20
  300, //down_duration_ms
  130  //up_duration_ms
};

GaitRobot trot_gait[4] = {
  { gait_trot,   0, NULL, 0, {25, 0, 0} },
  { gait_trot, 180, NULL, 0, {45, 0, 0} },
  { gait_trot, 180, NULL, 0, {25, 0, 0} },
  { gait_trot,   0, NULL, 0, {45, 0, 0} },
};

GaitLegParam gait_walk = {
  //amplitude, center, phase offset, clipmin
  { {30,   0,  0,  -100, 100 }, //x
    {20, 190, 90,   -90,  20 }, //y
    {0,    0,  0,  -100, 100 }  //z
  },
  290, //touchdown_start_deg //270 + 20
  70, //touchdown_end_deg    //90 - 20
  600, //down_duration_ms
  300  //up_duration_ms
};


//legs raise in order 3,2,1,0
//FL, BR, FR, BL

//BR raises at degrees 12, need cg to be on left side z offset shud be +50
//FR raises at degrees 72 or 84, need cg on to stay/be left side
//BL raises at degrees 168, need cg to SWITCH to right side quickly, z offset shud become -50
//FL raises at degrees 252, need cg to stay/be on right side, -50
//BR raises at degrees 0, need cg to be on left side, switch z offset to +50

//evaluate making 3 sep arrays for x y and z, then need
//not define those that aren't needed
/*
  CGOffset walk_cg_offsets[30] = {
  { 5, 0, 20 }, //0 degrees, back right is rising
  { 5, 0, 30 }, //12
  { 0, 0, 40 }, //24
  { 0, 0, 40 }, //36
  { 0, 0, 40 }, //48
  { 0, 0, 40 }, //60
  { 0, 0, 40 }, //72
  {-5, 0, 40 }, //84 //FR front right rises
  {-5, 0, 40 }, //96
  {-5, 0, 40 }, //108
  { 0, 0, 40 }, //120
  { 0, 0, 30 }, //132
  { 0, 0, 20 }, //144
  { 0, 0,  0 }, //156
  { 5, 0,-20 }, //168 BL back left rises
  { 5, 0,-30 }, //180
  { 5, 0,-40 },  //192
  { 0, 0,-40 }, //204
  { 0, 0,-40 }, //216
  { 0, 0,-40 }, //228
  { 0, 0,-40 }, //240
  {-5, 0,-40 }, //252 FL front left rises
  {-5, 0,-40 }, //264
  {-5, 0,-40 }, //276
  { 0, 0,-40 }, //288
  { 0, 0,-40 }, //300
  { 0, 0,-40 }, //312
  { 0, 0,-30 }, //324
  { 0, 0,-20 }, //336
  { 0, 0,  0 }, //348
  };
*/
//CGOffset walk_cg_offsets[30] = {
signed char walk_cg_offsets_x[30] = { 5, 5, 0, 0, 0, 0, 0, -5, -5, -5, 0, 0, 0, 0, 5, 5, 5, 0, 0, 0, 0, -5, -5, -5, 0, 0, 0, 0, 0, 0 };
//char walk_cg_offsets_y[30] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
signed char walk_cg_offsets_z[30] = { 20, 30, 40, 40, 40, 40, 40, 40, 40, 40, 40, 30, 20, 0, -20, -30, -40, -40, -40, -40, -40, -40, -40, -40, -40, -40, -40, -30, -20, 0};
signed char* walk_cg_offsets[3] = {walk_cg_offsets_x, NULL, walk_cg_offsets_z };

GaitRobot walk_gait[4] = {
  { gait_walk, 0, walk_cg_offsets, 30, {25, 0, 0} }, //FL
  { gait_walk,  90, NULL, 0, {45, 0, 0} }, //BL
  { gait_walk, 180, NULL, 0, {25, 0, 0}}, //FR
  { gait_walk, 270, NULL, 0, {45, 0, 0} }, //BR
};

GaitRobot* robot_gait = NULL;
#define gait_step 12 //12 degrees per cycle
signed char turn_right = 0;//more positive for more right, negative for turning left
signed char shuffle_right = 0;//more positive for more right, negative for turning left
bool gait_direction = true;


void resetGaits()
{
  for (char i = 0; i < 4; i++ )
  {
    trot_gait[i].gait = gait_trot;
    walk_gait[i].gait = gait_walk;
    turn_right = 0;
    shuffle_right = 0;
    gait_direction = true;
  }
}

class Leg
{
  public:
    char leg_id;
    float target_pos[3];
    float start_pos[3];
    float cur_pos[3];

    int index = 0;
    unsigned long start_time = 0;
    unsigned long end_time = 0;

    void setup( char id )
    {
      pinMode( OEPIN, OUTPUT ); //connected on OE/output-enable on servo driver
      digitalWrite( OEPIN, HIGH ); //high means off
      //ultrasonic sensor
      pinMode( A0, INPUT );
      pinMode( A1, OUTPUT );

      leg_id = id;
      for ( char i = 0; i < 3; i++ )
      {
        cur_pos[i] = leg_info[id].init_pos[i]; // + 10; //so incr isnt zero
        start_pos[i] = cur_pos[i];
        target_pos[i] = cur_pos[i];
      }
    }
    void print( bool include_offset )
    {
      Serial.print( (int)leg_id );
      for (char i = 0; i < 3; i++ )
      {
        Serial.print( "\t" );
        Serial.print( cur_pos[i] + (include_offset ? offsets[leg_id][i] : 0));
      }
      Serial.println();
    }
    void goTo( float* pos, int duration, bool reversed, unsigned long curtime ) //in milliseconds
    {
      for ( char i = 0; i < 3; i++ )
        if ( isnan( pos[i] ) || isinf(pos[i]) || !isValidAngle(pos[i] ) )
          return; //don't do it

      start_time = curtime;
      end_time = curtime + duration;
      for ( char i = 0; i < 3; i++ )
      {
        start_pos[i] = cur_pos[i];
        if ( i == 0 )
          target_pos[i] = ( leg_id == 1 || leg_id == 3 ? -pos[i] : pos[i] );
        else
          target_pos[i] = ( reversed ? -pos[i] : pos[i] );

        //check out of bounds
        int tpos = target_pos[i] + leg_info[leg_id].offset_pos[i];
        if ( tpos < -85 )
          target_pos[i] = -85  - leg_info[leg_id].offset_pos[i];
        else if ( tpos > 85 )
          target_pos[i] = 85  - leg_info[leg_id].offset_pos[i];
      }
    }
    void goTo( int* pos, int duration, bool reversed, unsigned long curtime ) //in milliseconds
    {
      float fpos[3] = { pos[0], pos[1], pos[2] };
      goTo( fpos, duration, reversed, curtime );
    }

    void goTo( float* pos, int duration, bool reversed, unsigned long curtime, bool is_xyz ) //in milliseconds
    {
      if ( is_xyz )
      {
        float angpos[3];
      //do stuff here   
        if ( calcAnglesXYZ( pos[0] + offsets[leg_id][0],
                            pos[1] + offsets[leg_id][1] + imu_offsets[leg_id],
                            pos[2] + offsets[leg_id][2],
                            angpos[0], angpos[1], angpos[2] ) )
          goTo( angpos, duration, reversed, curtime );
        else if ( flags.isset( FLAG_DEBUG) )
          println( "E", 0 );
      }
      else
      {
        goTo( pos, duration, reversed, curtime );
      }
    }
    void goTo( int* pos, int duration, bool reversed, unsigned long curtime, bool is_xyz ) //in milliseconds
    {
      float fpos[3] = { pos[0], pos[1], pos[2] };
      goTo( fpos, duration, reversed, curtime, is_xyz );
    }
    void goTo( signed char* pos, int duration, bool reversed, unsigned long curtime, bool is_xyz ) //in milliseconds
    {
      float fpos[3] = { pos[0], pos[1], pos[2] };
      goTo( fpos, duration, reversed, curtime, is_xyz );
    }

    void updateTarget( float* new_target, unsigned long curtime )
    {
      //if doing a motion, only update target
      //else send to new target
      if ( end_time == 0 || curtime > end_time )
      {
        goTo( new_target, 250, leg_info[leg_id].flags.isset(REVERSED), curtime, true );
      }
      else
      {
        //currently moving, update target without starting new motion
        goTo( new_target, end_time - start_time, leg_info[leg_id].flags.isset(REVERSED), start_time, true );
      }
    }
    void directGoto( float* pos )
    {
      for ( char j = 0; j < 3; j++ )
      {
        if ( pos[j] < -500 || pos[j] < -150 || pos[j] > 150 )
          return; //ignore this motor
        servogoto( leg_info[leg_id].ids[j], leg_info[leg_id].offset_pos[j] + pos[j] );
        cur_pos[j] = pos[j];
        target_pos[j] = pos[j];
      }
    }
    void directGoto( int* pos )
    {
      float fpos[3] = { pos[0], pos[1], pos[2] };
      directGoto( fpos );
    }
    void loop( unsigned long curtime )
    {
      if (end_time == 0 )
        return;
      for ( char i = 0; i < 3; i++ )
      {
        //desired position at this time in the cycle
        cur_pos[i] = ( float(curtime - start_time ) /
                       (float)(end_time - start_time) * (float)(target_pos[i] - start_pos[i]) ) +
                     (float)start_pos[i];
      }

      for ( char i = 0; i < 3; i++ )
      {
        int pos = cur_pos[i] + leg_info[leg_id].offset_pos[i]; //add the installation error adjustment
        servogoto( leg_info[leg_id].ids[i], pos );
      }
      if ( curtime > end_time )
      {
        end_time = 0; //done, mark motion as complete
      }
    }
    bool calcAngles( float x, float y, float& h, float& k ) //from xy position
    {
      //Serial.print( "x: " );
      //Serial.println( x);
      //Serial.print( "y: " );
      //Serial.println( y );

      double max_length = LTHIGH + LCALF;

      //printf( "x: %0.1lf, y:%0.1lf\r\n", x, y );
      double l = sqrt(x * x + y * y); //length of leg in desired  position
      if ( l > max_length )
        l = max_length;
      //printf( "length: %0.1lf\r\n", l );
      //now upper leg and lower leg and l form a triangle
      //this is angle from l, not vertical
      double l_hip = getangleabc( LTHIGH, l, LCALF);
      //printf( "l_hip: %0.1lf\r\n", l_hip );
      double l_angle = getangleabc( l, y, x );
      if ( x < 0 )
        l_angle = -l_angle;
      //printf( "l_angle: %0.1lf\r\n", l_angle );
      double hip_angle = l_hip + l_angle;

      //knee angle
      double knee_angle = 180.0 - getangleabc( LCALF, LTHIGH, l );

      //Serial.print( "hip angle: " );
      //Serial.println( hip_angle );
      //Serial.print( "knee angle: " );
      //Serial.println( knee_angle );

      h = hip_angle;
      k = knee_angle;
      float p = h + leg_info[leg_id].offset_pos[1];

      if ( p < -85 or p > 85 )
      {
        //if ( debug )
        //    Serial.println( "E3" );
        return false;
      }

      p = (leg_info[leg_id].flags.isset(REVERSED) ? -k : k) + leg_info[leg_id].offset_pos[2];
      /*
        if ( debug )
        {
        println( "h", h );
        println( "k", k );
        }

        if ( debug && !( p >= -85 and p <= 85 ) )
        {
        Serial.println( "E4" );
        println( "k", k );
        println( "p", p );
        }
      */
      return ( p >= -85 and p <= 85 );

      //clip( hip_angle, -60.0, 80.0 );
      //clip( knee_angle, -90.0, 135.0 );
    }
    bool calcAnglesXYZ( float x, float y, float z, float& r, float& h, float& k ) //from xyz position
    {
      float max_length = LTHIGH + LCALF;
      if ( x < -max_length || x > max_length || y < 0 || y > max_length ||
           z < -150 || z > 150 )
      {
        if ( flags.isset( FLAG_DEBUG) )
          println( "E", 1 );
        return false;
      }
      double oplusz = ROTRADIUS + z;
      double dsq = y * y + oplusz * oplusz;
      double d = sqrt( dsq );
      double cosa = oplusz / d;
      double a = acos( cosa );
      double cosaplusr = ROTRADIUS / d;
      double aplusr = acos( cosaplusr );

      r = aplusr - a;//in radians
      if ( isnan(r ) )
      {
        if ( flags.isset( FLAG_DEBUG) )
          println( "E", 2 );
        return false;
      }

      r = r * RAD_TO_DEG;
      r = -r;


      double lsq = dsq - ROTRADIUS * ROTRADIUS;
      double l = sqrt( lsq );
      //if ( debug )
      //  println( "r", r );
      //now the 2d part
      return calcAngles( x, l, h, k );
    }

    void calcPosition( float hip_angle, float knee_angle, float& x, float& y ) //from angles
    {
      knee_angle = 180.0 - knee_angle;
      //find height of knee below hip
      double sa_rad = hip_angle * DEG_TO_RAD;
      double yk = LTHIGH * cos( sa_rad );
      double xk = LTHIGH * sin( sa_rad  );

      //find height of foot below knee
      double ak_rad = (180.0 - hip_angle - knee_angle) * DEG_TO_RAD;
      double yf = LCALF * cos( ak_rad );
      double xf = LCALF * sin( ak_rad );

      y = -yk - yf;

      double xz = xk - xf;

      //now find x and z from xz
      //double wa_rad = waist_angle * deg2rad;
      x = xz; // * cos( wa_rad);
      //z = 0; //-xz * sin( wa_rad);

      y = -y;
    }
    void calcPositionXYZ(  float rot_angle,
                           float hip_angle,
                           float knee_angle,
                           float& x, float& y, float& z ) //from angles
    {
      knee_angle = 180.0 - knee_angle;
      //find height of knee below hip
      double sa_rad = hip_angle * DEG_TO_RAD;
      double yk = LTHIGH * cos( sa_rad );
      double xk = LTHIGH * sin( sa_rad  );

      //find height of foot below knee
      double ak_rad = (180.0 - hip_angle - knee_angle) * DEG_TO_RAD;
      double yf = LCALF * cos( ak_rad );
      double xf = LCALF * sin( ak_rad );

      y = -yk - yf;

      double xz = xk - xf;

      //now find x and z from xz
      //double wa_rad = waist_angle * deg2rad;
      x = xz; // * cos( wa_rad);
      //z = 0; //-xz * sin( wa_rad);

      y = -y;

      //this is x and y assuming leg was vertical with zero rot_angle
      //when rotator rotates, x remains the same, only y changes
      double r = rot_angle * DEG_TO_RAD;
      z = ( ROTRADIUS * cos(r) ) - ( y * sin(r) );
      z -= ROTRADIUS; //remove effect of rot_radius
      //update y
      y = y * cos(r) + ROTRADIUS * sin(r);

    }
    void getPosition( float* pos, bool target, bool remove_offsets ) //from angles
    {
      bool reversed = leg_info[leg_id].flags.isset(REVERSED);
      if ( !target )
      {
        pos[0] = cur_pos[0];
        calcPosition( ( reversed ? -cur_pos[1] : cur_pos[1] ),
                      ( reversed ? -cur_pos[2] : cur_pos[2] ),
                      pos[1], pos[2] );
      }
      else
      {
        pos[0] = target_pos[0];
        calcPosition( (reversed ? -target_pos[1] : target_pos[1] ),
                      (reversed ? -target_pos[2] : target_pos[2] ),
                      pos[1], pos[2] );
      }
      //fix for left right back forward reversing of motor angles
      pos[0] = ( leg_id == 1 || leg_id == 3 ? -pos[0] : pos[0] );

      if ( remove_offsets )
        for ( char i = 0; i < 3; i++ )
          pos[i] -= offsets[leg_id][i];

    }
    void getPositionXYZ( float* pos, bool target, bool remove_offsets ) //from angles
    {
      /*
        if ( j == 0 )
            {
              if ( l == 1 || l == 3 )
                apos = -apos;
            }
        else if ( leg_info[l].reversed )
              apos = -apos;
      */
      bool reversed = leg_info[leg_id].flags.isset(REVERSED);
      if ( !target )
      {
        calcPositionXYZ( ( (leg_id == 1 || leg_id == 3) ? -cur_pos[0] : cur_pos[0] ),
                         ( reversed ? -cur_pos[1] : cur_pos[1] ),
                         ( reversed ? -cur_pos[2] : cur_pos[2] ),
                         pos[0], pos[1], pos[2] );
      }
      else
      {
        calcPositionXYZ( ((leg_id == 1 || leg_id == 3)  ? -target_pos[0] : target_pos[0] ),
                         (reversed ? -target_pos[1] : target_pos[1] ),
                         (reversed ? -target_pos[2] : target_pos[2] ),
                         pos[0], pos[1], pos[2] );
      }
      //fix for left right back forward reversing of motor angles
      //pos[0] = ( leg_id == 1 || leg_id == 3 ? -pos[0] : pos[0] );

      if ( remove_offsets )
        for ( char i = 0; i < 3; i++ )
          pos[i] -= offsets[leg_id][i];

    }
};

struct LegStep
{
  int pos[3];
  int duration;
  bool is_xyz; //must be converted before use
};

struct LegMotion
{
  LegStep* legsteps;
  char len;
  signed char offset;
};

struct BodyMotion
{
  bool continuous;
  LegMotion leg_motion[4];
};


signed char speed_factor10 = 10; //save three bytes, have to divide by 10 everywhere

float speedFactor()
{
  return (float)speed_factor10 / 10.0;
}

class Body
{
  public:
    Leg legs[NUM_LEGS];
    BodyMotion* cur_body_motion = NULL;
    GaitRobot* cur_gait = NULL;
    bool manual_stepping_mode = false;
    bool manual_step_allowed = false;

    void init()
    {
      pinMode(BARK_PLAY_E, OUTPUT);
      unsigned long curtime = millis();
      for ( char i = 0; i < NUM_LEGS; i++ )
      {
        legs[i].setup( i );
        legs[i].goTo( leg_info[i].init_pos, 1000, leg_info[i].flags.isset(REVERSED), \
                      curtime, leg_info[i].flags.isset( IS_XYZ ) );
      }

      legs[0].index = 0;
      legs[1].index = 4;
      legs[2].index = 2;
      legs[3].index = 6;
    }
    void stop()
    {
      cur_gait = NULL;
      robot_gait = NULL;
      cur_body_motion = NULL;
      gait_direction = true;
      for (char i = 0; i < 4; i++ )
        legs[i].end_time = 0; //make it stop
    }
    void beginGait( GaitRobot* gait, bool direction )
    {
      cur_body_motion = NULL;

      cur_gait = gait;
      gait_direction = direction;
      for (char l = 0; l < 4; l++ )
      {
        legs[l].index = cur_gait[l].phase_offset;
        for (char j = 0; j < 3; j++ ) //reset gait offset
          gait_offsets[l][j] = 0;
      }
    }
    void beginMotion( struct BodyMotion* body_motion )
    {
      cur_gait = NULL;
      robot_gait = NULL;
      for (char i = 0; i < 4; i++ ) {
        legs[i].index = (body_motion->continuous ? body_motion->leg_motion[i].offset - 1 : -1) ;
      }
      cur_body_motion = body_motion;
      /*
        if ( debug )
        Serial.println( name );
      */
    }
    bool loop() //returns true if something moved
    {
      //mouth
      char mouth_incr = 1;
      if ( flags.isset( FLAG_BARK) )
        mouth_incr = 3;
      if ( mouth_target > mouth_pos )
        mouth_pos += mouth_incr;
      else if ( mouth_target < mouth_pos )
        mouth_pos -= mouth_incr;
      if ( flags.isset( FLAG_BARK ) )
        if ( mouth_pos >= MOUTH_OPEN_MAX )
          mouth_target = MOUTH_CLOSE;
      servogoto( 12, mouth_pos );
      unsigned long curtime = millis();
      bool none_moving = true;
      float speed_factor = speedFactor();

      if(flags.isset(FLAG_IMU) && cur_gait){
        runIMU(true);
      }
      
      for ( char i = 0; i < 4; i++ )
      {
        //Serial.print( "Leg index: " );
        //Serial.println( i );
        if ( legs[i].end_time == 0 ||
             legs[i].end_time <= curtime
           )
        { //move to next step
          if ( cur_gait && (!manual_stepping_mode || manual_step_allowed) )
          {

            //float gait_step_ms = GAIT_STEP_MS * (float)(cur_gait[i].gait.xyz[0].amplitude +
            //                                    cur_gait[i].gait.xyz[1].amplitude +
            //                                    cur_gait[i].gait.xyz[2].amplitude )  / 30.0;
            float gait_step_ms = (float)cur_gait[i].gait.getTotalDuration() / (float)gait_step;
            if ( i == 0 && flags.isset( FLAG_DEBUG) )
            {
              Serial.print( "deg\t" );
              Serial.print( legs[i].index );
            
//              Serial.println( "gait_step_ms", gait_step_ms );
            }

            int duration = gait_step_ms; //constant duration, but increased X and Z stride
            float pos[3];
            int vdeg = cur_gait[i].gait.getPosition( legs[i].index, pos, speed_factor );
            //if a cg offset is defined by this step, then execute it
            //but how, apply to global offsets, or add another cg offset global
            //todo: ideally need separate global, but for now modify regular offsets
            if ( cur_gait[i].num_cg_offsets > 0 )
            {
              signed char cycle_index = (float)legs[i].index / 360.0 * cur_gait[i].num_cg_offsets;
              for ( char l = 0; l < 4; l++ )
              {
                for (char j = 0; j < 3; j++ )
                {
                  if ( cur_gait[i].cg_offsets[j] )
                    gait_offsets[l][j] = cur_gait[i].cg_offsets[j][cycle_index]; //.o[j];
                }
              }
            }
            for (char j = 0; j < 3; j++ )
            {
              pos[j] += cur_gait[i].main_offset[j] + gait_offsets[i][j];
            }
            if ( i == 0 && flags.isset( FLAG_DEBUG) )
            {
              for ( char k = 0; k < 3; k++ )
              {
                Serial.print( "\t" );
                Serial.print( pos[k] );
              }
              Serial.print( "\t" );
              Serial.println( vdeg );
            }
            /*
              if ( debug && i == 0 )
              {
              println( "Deg", legs[i].index );
              for(int k = 0; k < 3; k++ )
                Serial.println( pos[k] );
              }
            */
            legs[i].goTo( pos, duration, leg_info[i].flags.isset(REVERSED),
                          curtime, true );
            none_moving = false;

            legs[i].index += (gait_direction ? gait_step : -gait_step );
            if ( legs[i].index >= 360 )
              legs[i].index -= 360;
            if ( legs[i].index < 0 )
              legs[i].index += 360;

            continue;
          }
          if ( !cur_body_motion )
            continue;


          
          legs[i].index++;
          LegMotion& leg_motion = cur_body_motion->leg_motion[i];
          if ( cur_body_motion->continuous && legs[i].index >= leg_motion.len)
          {
            legs[i].index = 0;
            legs[i].start_time = curtime;
          }
          if ( leg_motion.legsteps != NULL &&
               legs[i].index < leg_motion.len ) //not done
          {
            if ( legs[i].index < 0 )
              legs[i].index = 0;
            int index = legs[i].index;
            {
              //Serial.print( "Leg index: " );
              //Serial.println( index );
            }
            
            int duration = ((float)leg_motion.legsteps[index].duration / speed_factor);

            
            legs[i].goTo( leg_motion.legsteps[index].pos, duration, leg_info[i].flags.isset(REVERSED),
                          curtime, leg_motion.legsteps[index].is_xyz );
            none_moving = false;
          }
        }
        else
        {
          none_moving = false;
          legs[i].loop( curtime);
        }
        manual_step_allowed = false;
      }

      if ( none_moving )
      {
        //Serial.println( "Motion done" );
        cur_body_motion = NULL;
      }
      return (!none_moving);
    }

};

Body body;

void setup() {
  Serial.begin(500000);
  Serial.println();
  Wire.begin();
  servo_driver.setup();
  //Serial << robot_pose << "\n";
  body.init();
  delay(2000);
  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
   }
}

//front-left, back-left, front-right, back-right


#define DownPos  { 0, 160, 0 } //crouch
#define RestPos  { 0, 200, 0 } //is_xy in mm
#define TallPos  { 0, 0,  0}

#define SitPos { 0, 10, 100 }

LegStep Crouch[] = {
  { DownPos, 800, true },
  { DownPos, 500, true }, //stay there
  { RestPos, 800, true },
};

LegStep Balance[] = {
  { DownPos, 1000, true },
};

LegStep Sit[] = {
  { DownPos, 1000, true },
  { SitPos,  1800, false },
};

LegStep Stand[] = {
  { RestPos, 1000, true },
};

#define GetupPos { 0, 50, 120 }

LegStep Getup[] = {
  { GetupPos, 1200, false },
  { DownPos, 500, true },
  { RestPos, 1000, true }
};

LegStep Tall[] = {
  { TallPos, 1000, false }
};

//need a way to specify diff sequences for diff legs
//and also a "wait at current" position position

BodyMotion BodyBalance = 
{
  false,
  {
    { Balance, asize(Balance) },
    { Balance, asize(Balance) },
    { Balance, asize(Balance) },
    { Balance, asize(Balance) },
  }
};

BodyMotion BodyCrouch =
{
  false,
  {
    { Crouch, asize(Crouch) },
    { Crouch, asize(Crouch) },
    { Crouch, asize(Crouch) },
    { Crouch, asize(Crouch) },
  }
};

BodyMotion BodySit =
{
  false,
  {
    { Sit, asize(Sit) },
    { Sit, asize(Sit) },
    { Sit, asize(Sit) },
    { Sit, asize(Sit) },
  }
};

BodyMotion BodyStand =
{
  false,
  {
    { Stand, asize(Stand) },
    { Stand, asize(Stand) },
    { Stand, asize(Stand) },
    { Stand, asize(Stand) },
  }
};
BodyMotion BodyTall =
{
  false,
  {
    { Tall, asize(Tall) },
    { Tall, asize(Tall) },
    { Tall, asize(Tall) },
    { Tall, asize(Tall) },
  }
};
BodyMotion BodyGetup =
{
  false,
  {
    { Getup, asize(Getup) },
    { Getup, asize(Getup) },
    { Getup, asize(Getup) },
    { Getup, asize(Getup) },
  }
};

BodyMotion BodyBow =
{
  false,
  {
    { Crouch, asize(Crouch) },
    { NULL, 0 },
    { Crouch, asize(Crouch) },
    { NULL, 0 },
  }
};

/*
  int xOffset = 20;     // move center of step back
  int stepFactor = 10;  // default 10


  LegStep WalkL[] = {

  //support (on-ground) phase should be 3/4 of total time
  //in-air, moving forward phase should be 1/4 of total time

  //lets say 2400 is total time
  //then support phase is 1800
  //and move phase is 600
  //also each leg is 600 apart
  //600, 600, 600, [200,200,200]
  //x positive is backward, y reduces upwards

  { {  4 * stepFactor + xOffset, 190,  0  }, 200, true }, // 0 lift
  { {  0 * stepFactor + xOffset, 190,  0  }, 400, true }, // 1 pass

  { { -4 * stepFactor + xOffset, 190, 10 }, 400, true }, // 2 down
  { { -6 * stepFactor + xOffset, 210, 10 }, 200, true }, // 3 contact start
  { { -3 * stepFactor + xOffset, 210, 10 }, 200, true }, // 4

  { { -2 * stepFactor + xOffset, 210, 20 }, 200, true }, // 5
  { {  0 * stepFactor + xOffset,  210,20 }, 200, true }, // 6 mid

  { {  2 * stepFactor + xOffset, 210,  0 }, 200, true }, // 7
  { {  4 * stepFactor + xOffset, 210,-20 }, 200, true }, // 8

  { {  6 * stepFactor + xOffset, 210,-20 }, 200, true }, // 9 contact end
  //r smaller means in

  };
  LegStep WalkR[] = {

  { {  4 * stepFactor + xOffset, 190, -5 }, 200, true }, // 0 lift
  { {  0 * stepFactor + xOffset, 200, -10 }, 400, true }, // 1 pass

  { { -4 * stepFactor + xOffset, 190,  -5 }, 400, true }, // 2 down
  { { -6 * stepFactor + xOffset, 210,  0  }, 200, true }, // 3 contact start
  { { -3 * stepFactor + xOffset, 210,  0  }, 200, true }, // 4

  { { -2 * stepFactor + xOffset, 210, 5   }, 200, true }, // 5
  { {  0 * stepFactor + xOffset, 210, 10  }, 200, true }, // 6 mid

  { {  2 * stepFactor + xOffset, 210, 5   }, 200, true }, // 7
  { {  4 * stepFactor + xOffset, 210, 0   }, 200, true }, // 8

  { {  6 * stepFactor + xOffset, 210, 0   }, 200, true }, // 9 contact end
  };


  LegStep WalkL[] = {
  /*
  { {14,  177,   15}, 200, true}, //FL begin lift
  { {7, 170,     15}, 200, true},
  { {1, 170,     15}, 200, true},
  { {-5,  170,   15}, 200, true},
  { {-11, 174,   15}, 200, true},
  { {-18, 185,    5}, 200, true},
  { {-24, 192,    0}, 200, true},
  { {-20, 200,   -15}, 200, true},
  { {-10, 200,   -15}, 200, true},
  { {0,   200,   -15}, 200, true},
  { {10,  200,   -5}, 200, true},
  { {20,  200,    0}, 200, true},

  { {26,  170,   0}, 200, true},
  { {-11, 170,   0}, 200, true},
  { {-36, 185,   0}, 200, true},
  { {-30, 200,   0}, 200, true},
  { {-23, 200,   0}, 200, true},
  { {-15, 200,   0}, 200, true},
  { {-8,  200,   0}, 200, true},
  { {0, 200,   0}, 200, true},
  { {8, 200,   0}, 200, true},
  { {15,  200,   0}, 200, true},
  { {23,  200,   0}, 200, true},
  { {30,  200,   0}, 200, true},

  };

  LegStep WalkR[] = {

  { {14,  177,   -15}, 200, true}, //FL begin lift
  { {7, 170,     -15}, 200, true},
  { {1, 170,     -15}, 200, true},
  { {-5,  170,   -15}, 200, true},
  { {-11, 174,   -15}, 200, true},
  { {-18, 185,    -5}, 200, true},
  { {-24, 192,    0}, 200, true},
  { {-20, 200,   15}, 200, true},
  { {-10, 200,   15}, 200, true},
  { {0,   200,   15}, 200, true},
  { {10,  200,   5}, 200, true},
  { {20,  200,    0}, 200, true},

  { {26,  170,   0}, 200, true},
  { {-11, 170,   0}, 200, true},
  { {-36, 185,   0}, 200, true},
  { {-30, 200,   0}, 200, true},
  { {-23, 200,   0}, 200, true},
  { {-15, 200,   0}, 200, true},
  { {-8,  200,   0}, 200, true},
  { {0, 200,   0}, 200, true},
  { {8, 200,   0}, 200, true},
  { {15,  200,   0}, 200, true},
  { {23,  200,   0}, 200, true},
  { {30,  200,   0}, 200, true},
  };
  BodyMotion BodyWalk =
  {
   true,
   {
    //fl, bl, fr, br
    { WalkL, asize(WalkL), 0, true },
    { WalkL, asize(WalkL), 3, true },
    { WalkR, asize(WalkR), 6, true },
    { WalkR, asize(WalkR), 9, true },

    // rear delay = 1/2

    { WalkL, asize(WalkL), 7, true },
    { WalkL, asize(WalkL), 2, true },
    { WalkR, asize(WalkR), 2, true },
    { WalkR, asize(WalkR), 7, true },

    // rear delay = 3/4 - back lead
    { WalkL, asize(WalkL), 7, true },
    { WalkL, asize(WalkL), 5, true },
    { WalkR, asize(WalkR), 2, true },
    { WalkR, asize(WalkR), 9, true },

    // rear delay = 1/4 - front lead
    { WalkL, asize(WalkL), 9, true },
    { WalkL, asize(WalkL), 2, true },
    { WalkR, asize(WalkR), 5, true },
    { WalkR, asize(WalkR), 7, true },
   }
  };

*/
struct CmdMotion
{
  const char* cmd;
  BodyMotion* motion;
};
CmdMotion commands[] = {
  { "crouch", &BodyCrouch },
  { "sit", &BodySit },
 // { "stand", &BodyStand }, //now executes getup first if too low
  { "getup", &BodyGetup },
  { "tall", &BodyTall },
  { "bow", &BodyBow },
  {"balance", &BodyBalance },
};

bool legIdMatches( char l, char leg_id )
{
  return ( (leg_id < 4 && l == leg_id ) || //individual leg
           leg_id == 8 || //all
           (leg_id == 4 && (l == 0 || l == 2 ) ) || //front
           (leg_id == 5 && (l == 1 || l == 3 ) ) || //back
           (leg_id == 6 && (l == 0 || l == 1 ) ) || //left
           (leg_id == 7 && (l == 2 || l == 3 ) )    //right
         );
}

void adjustPos( float* pos, const char* jcmd, float incr )
{
  if ( !strcmp( jcmd, "x" ) )
    pos[0] -= incr;
  else if ( !strcmp (jcmd, "X") )
    pos[0] += incr;
  else if ( !strcmp (jcmd, "y") )
    pos[1] -= incr;
  else if ( !strcmp (jcmd, "Y") )
    pos[1] += incr;
  else if ( !strcmp (jcmd, "z") )
    pos[2] -= incr;
  else if ( !strcmp (jcmd, "Z") )
    pos[2] += incr;
}

void adjustPos( signed char* pos, const char* jcmd, float incr )
{
  if ( !strcmp( jcmd, "x" ) )
    pos[0] -= incr;
  else if ( !strcmp (jcmd, "X") )
    pos[0] += incr;
  else if ( !strcmp (jcmd, "y") )
    pos[1] -= incr;
  else if ( !strcmp (jcmd, "Y") )
    pos[1] += incr;
  else if ( !strcmp (jcmd, "z") )
    pos[2] -= incr;
  else if ( !strcmp (jcmd, "Z") )
    pos[2] += incr;
}
void adjustXYZPos( char leg_id, const char* jcmd, float incr )
{
  float pos[3];
  body.legs[leg_id].getPositionXYZ( pos, true, true );
  adjustPos( pos, jcmd, incr );
  body.legs[leg_id].goTo( pos, 50, leg_info[leg_id].flags.isset(REVERSED), millis(), true );
}

void adjustAngles( char leg_id, const char* jcmd, float increment )
{
  float pos[3];
  for (char i = 0; i < 3; i++ )
    pos[i] = (i == 0 && (leg_id == 1 || leg_id == 3) ? -body.legs[leg_id].cur_pos[i] : body.legs[leg_id].cur_pos[i] );
  signed char incr = (leg_id == 2 || leg_id == 3 ? -increment : increment );
  if ( !strcmp( jcmd, "r" ) )
    pos[0] -= increment;
  else if ( !strcmp (jcmd, "R") )
    pos[0] += increment;
  else if ( !strcmp (jcmd, "h") )
    pos[1] -= incr;
  else if ( !strcmp (jcmd, "H") )
    pos[1] += incr;
  else if ( !strcmp (jcmd, "k") )
    pos[2] -= incr;
  else if ( !strcmp (jcmd, "K") )
    pos[2] += incr;
  body.legs[leg_id].goTo( pos, 50, false, millis() ); //don't reverse again
}

void startGait( GaitRobot* gait, bool direction )
{
  //todo, need to add reset gait somewhere
  //and verify speed and backward operation at zero boundary
  //also add gait height control

  if ( robot_gait == gait ) //already walking
  {
    if ( direction == gait_direction )
      incrGaitAmplitude( robot_gait, 0, 5 ); //x
    else
    {
      int new_amplitude = robot_gait[0].gait.xyz[0].amplitude - 5;
      if ( (robot_gait[0].gait.xyz[0].amplitude >= 0 && new_amplitude < 0) )
        gait_direction = false;
      else if (robot_gait[0].gait.xyz[0].amplitude < 0 && new_amplitude > 0)
        gait_direction = true;
      incrGaitAmplitude( robot_gait, 0, -5 ); //x
    }
  }
  else
  {
    robot_gait = gait;
    //get direction if specified
    body.beginGait( robot_gait, direction );
  }
}

unsigned long last_ultrasonic_when = 0;
int ultrasonic_distance = -1;

void loop() {
  // put your main code here, to run repeatedly:
  if ( Serial.available() > 0 )
  {
    //Serial.println("command called");
    unsigned long curtime = millis();

    char cmd[96];
    char c = Serial.read();
    int len = 0;
    while ( c != '\n' )
    {
      if ( c != -1 )
        cmd[len++] = c;
      //Serial.print( c );
      while ( !Serial.available() )
        delayMicroseconds( 10 );
      c = Serial.read();
    }
    cmd[len] = 0;
    //Serial.println( len );
    //Serial.println( cmd );
    if ( !strcmp( cmd, "noop" ) )
    {
      //do nothing
    }
    else if (!strcmp(cmd, "imu")) {
      if(flags.isset(FLAG_IMU)) {
        flags.unset(FLAG_IMU);
        for (int i = 0; i < 4; i++ ) 
          imu_offsets[i] = 0;
      }  
      else 
        flags.set(FLAG_IMU);
    }
    else if ( !strcmp(cmd, "stop" ) )
    {
      flags.set( FLAG_PAUSED );
      body.cur_body_motion = NULL;
      body.cur_gait = NULL;
      robot_gait = NULL;
    }
    else if ( !strcmp(cmd, "go" ) )
    {
      flags.unset( FLAG_PAUSED );
    }
    else if ( !strcmp( cmd, "90" ) )
    {
      flags.set( FLAG_PAUSED );
      int zeropos[] = {0, 0, 0};
      for ( char i = 0; i < NUM_LEGS; i++ )
        body.legs[i].directGoto( zeropos );
    }
    else if ( !strcmp(cmd, "debugon" ) )
      flags.set( FLAG_DEBUG );
    else if ( !strcmp(cmd, "debugoff" ) )
      flags.unset( FLAG_DEBUG );
    else if ( len > 2 && cmd[0] == 'b' && cmd[1] == ':' ) //motor goto
    {
      int motor_id = atoi( cmd + 2 );
      int pos = 3;
      while ( pos < len && cmd[pos] != ',' )
        pos++;
      if ( cmd[pos] == ',' && len > (pos + 1) )
      {
        int deg = atof( cmd + pos + 1 );
        //Serial.print( motor_id);
        //Serial.print( " go to " );
        //Serial.println( deg );
        servogoto( motor_id, deg );
      }
    }
    else if ( !strcmp( cmd, "on" ) )
    {
      flags.set( FLAG_POWER );
      /*
        for( char i = 0; i < 10; i++ )
        {
        digitalWrite( 2, LOW );
        delay( 10 );
        digitalWrite( 2, HIGH );
        delay( 10 );
        }
      */
      digitalWrite( OEPIN, LOW );
    }
    else if ( !strcmp( cmd, "off" ) )
    {
      flags.unset( FLAG_POWER );
      for ( char i = 0; i < 13; i++ ) //include mouth servo
        servo_driver.stop( i );
      delay( 100 );
      for ( char i = 0; i < 10; i++ )
      {
        digitalWrite( OEPIN, LOW );
        delay( 10 );
        digitalWrite( OEPIN, HIGH );
        delay( 10 );
      }
      digitalWrite( OEPIN, HIGH );
    }
    else if ( !strcmp( cmd, "p" ) )
    {
      for (int i = 0; i < 4; i++ )
        body.legs[i].print( true );
    }
    else if ( !strcmp( cmd, "po" ) )
    {
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          Serial.print(int(offsets[i][j]));
          Serial.print(" ");
        }
        Serial.println();
      }
    }
    else if ( cmd[0] == 'j' && cmd[1] == ',' )
    {
      //j,1,Y  will increase Y for back left
      char leg_id = atoi(cmd + 2);
      const char* jcmd = cmd + 4;
      int incr = 5; //default
      if ( len > 6 && cmd[5] == ',' )
      {
        cmd[5] = 0;
        incr = atof( cmd + 6 );
      }
      for ( char l = 0; l < 4; l++ )
      {
        if ( legIdMatches( l, leg_id ) )
          adjustXYZPos( l, jcmd, incr );
      }
    }
    else if ( cmd[0] == 'k' && cmd[1] == ',' )
    {
      char leg_id = atoi(cmd + 2);
      const char* jcmd = cmd + 4;
      int incr = 3; //default
      if ( len > 6 && cmd[5] == ',' )
      {
        cmd[5] = 0;
        incr = atof( cmd + 6 );
      }
      for (char l = 0; l < 4; l++ )
      {
        if ( legIdMatches(l, leg_id ) )
          adjustAngles( l, jcmd, incr );
      }
    }
    else if ( cmd[0] == 'o' && cmd[1] == ',' )
    {
      //save target position before offsets
      float current_target[4][3];
      for ( char i = 0; i < 4; i++ )
        body.legs[i].getPositionXYZ( current_target[i], true, true );

      const char* ocmd = cmd + 2;
      if ( !strcmp( ocmd, "r" ) ) //reset all offsets
      {
        for (char i = 0; i < 4; i++ )
          for (char j = 0; j < 3; j++ )
            offsets[i][j] = 0;
      }
      else
      {
        char leg_id = atoi(ocmd);
        const char* jcmd = cmd + 4;
        int incr = 5; //default
        if ( len > 6 && cmd[5] == ',' )
        {
          cmd[5] = 0;
          incr = atof( cmd + 6 );
        }
        for (char l = 0; l < 4; l++ )
        {
          if ( legIdMatches(l, leg_id ) )
            adjustPos( offsets[l], jcmd, incr );
        }
      }
      //check if offsets go out of bounds

      //110 - 225

      int minY; 
      for (char i = 0; i < 4; i++){
        
      }
      
      //now offsets are changed
      for ( char i = 0; i < 4; i++ )
        body.legs[i].updateTarget( current_target[i], curtime );

      //print offsets
      /*
        if ( debug )
        {
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 3; j++) {
            Serial.print(int(offsets[i][j]));
            Serial.print(" ");
          }
          Serial.println();
        }
        Serial.println("---");
        }
      */
      
    }
    else if ( len > 2 && cmd[0] == 's' && cmd[1] == ':' ) //set speed factor
    {
      speed_factor10 = atof( cmd + 2 ) * 10.0;
      /*
        if ( debug )
        {
        Serial.print( "Setting speed factor to " );
        Serial.println( speed_factor );
        }
      */
    }
    else if ( len > 2 && cmd[0] == 'l' && cmd[1] == ':' ) //move leg to
    {
      //first val after : is the leg_id,
      //each of the three comma sep numbers after leg_id specifies the position
      //of rotation, hip, and knee
      char* vals[8];
      int n = split( cmd + 2, ',', 8, vals );
      if ( n > 3 )
      {
        int leg_id = atoi( vals[0] );
        float pos[3];
        for ( char i = 0; i < 3; i++ )
        {
          if ( vals[i + 1][0] == 0 ) //blank, ignore this motor, or leave it where it is
            pos[i] = -1000; //ignored if below -500
          else
            pos[i] = atof(vals[i + 1]);
        }
        body.legs[leg_id].directGoto( pos );
      }
    }
    else if ( len > 2 && cmd[0] == 'e' && cmd[1] == ':' ) //set all "tall" offsets to
    {
      //12 numbers, rotator, hip, knee for each leg
      //each number if new error_offset
      char* vals[14];
      int n = split( cmd + 2, ',', 14, vals );
      if ( n > 11 )
      {
        signed char i = 0;
        for (char l = 0; l < 4; l++ )
          for ( char j = 0; j < 3; j++ )
          {
            float offset_pos = atof( vals[i++] );
            if ( offset_pos > -100 and offset_pos < 100 )
            {
              //first adjust target and current pos of joint
              float change = offset_pos - leg_info[l].offset_pos[j];
              body.legs[l].cur_pos[j] -= change;
              body.legs[l].target_pos[j] -= change;
              leg_info[l].offset_pos[j] = offset_pos;
            }
          }
      }
    }

    /*
      else if ( len > 2 && cmd[0] == 'g' && cmd[1] == ':' ) //move leg to x, y
      {
       //each of the three comma sep numbers after : specifies the position
       //of rotation, hip, and knee
       char* vals[10];
       int n = split( cmd + 2, ',', 10, vals );
       if ( n > 2 )
       {
         int leg_id = atoi( vals[0] );
         float x = atof( vals[1] );
         float y = atof( vals[2] );
         float pos[] = { 0, 0, 0 };
         body.legs[leg_id].calcAngles( x, y, pos[1], pos[2] );
         body.legs[leg_id].directGoto( pos );
       }
      }
    */
    else if ( len == 1 && cmd[0] == 'i' ) //all info
    {
      //hmm..todo, should also send current speed factor. also replace "info" with "i"
      Serial.print( "info," );
      Serial.print( ( flags.isset( FLAG_POWER ) ? 1 : 0 ) );
      Serial.print( "," );
      Serial.print( ( flags.isset( FLAG_PAUSED ) ? 1 : 0 ) );
      Serial.print( "," );
      Serial.print( ultrasonic_distance );
      //Serial << "info,"
      //       << ( flags.isset( FLAG_POWER ) ? 1 : 0 ) <<  ","
      //       << ( flags.isset( FLAG_PAUSED ) ? 0 : 1 ) //enabled, opposite of paused
      //       << ","
      //       << ultrasonic_distance;
      for ( char l = 0; l < 4; l++ )
      {
        for ( char j = 0; j < 3; j++ )
        {
          float apos = body.legs[l].cur_pos[j];
          if ( j == 0 )
          {
            if ( l == 1 || l == 3 )
              apos = -apos;
          }
          else if ( leg_info[l].flags.isset(REVERSED) )
            apos = -apos;

          Serial.print( "," );
          Serial.print( round(apos) );
        }
      }
      Serial.println();
    }
    else if ( len == 1 && cmd[0] == 'u' ) //cur actual positions without offsets or sign reversal etc
    {
      Serial.print( "apos," );
      Serial.print( speedFactor() );
      for ( char l = 0; l < 4; l++ )
      {
        for ( char j = 0; j < 3; j++ )
        {
          float apos = body.legs[l].cur_pos[j] + leg_info[l].offset_pos[j];
          Serial.print( "," );
          Serial.print( round(apos) );
        }
      }
      Serial.println();
    }
    else if ( len == 1 && cmd[0] == 'v' )
    {
      Serial.println( FIRMWARE_VERSION );
    }
    else if ( len == 1 && cmd[0] == 't' ) //all info as xyz
    {
      Serial.print( "xyz," );
      Serial.print( (flags.isset( FLAG_POWER ) ? 1 : 0 ) );
      Serial.print( "," );
      Serial.print( (flags.isset( FLAG_PAUSED )  ? 0 : 1 ) ); //enabled, opposite of paused
      Serial.print( "," );
      Serial.print( ultrasonic_distance );

      for ( char l = 0; l < 4; l++ )
      {
        float cur_xyz[3];
        body.legs[l].getPositionXYZ( cur_xyz, false, true );
        for ( char j = 0; j < 3; j++ )
        {
          Serial.print( "," );
          Serial.print( round(cur_xyz[j]) );
        }
      }
      Serial.println();
    }
    else if ( len > 2 && cmd[0] == 'd' && cmd[1] == ',' ) //move leg to x, y, z over time duration
    {
      //d,<leg_id>,<duration_ms>,x,y,z
      char* vals[8];
      int n = split( cmd + 2, ',', 10, vals );
      if ( n > 3 )
      {
        char leg_id = atoi( vals[0] );
        int duration = atoi( vals[1] );
        float pos[3] = { atof( vals[2] ), //x
                         atof( vals[3] ), //y
                         ( n > 4 ? atof( vals[4] ) : 0 ) //z
                       };
        body.legs[leg_id].goTo( pos,
                                duration,
                                leg_info[leg_id].flags.isset(REVERSED),
                                curtime,
                                true );
      }
    }
    else if ( cmd[0] == 'w' && cmd[1] == ',' )
    { //w,duration,12 motors r,h,k angles for 4 legs
      unsigned long curtime = millis();
      char* vals[15];
      int n = split( cmd + 2, ',', 14, vals );
      if ( n >= 12 )
      {
        int duration = atoi( vals[0] );
        //vals[1] is 'r', or rotation of rotator
        float pos[3];
        for ( char i = 0; i < 4; i++ )
        {
          for ( char j = 0; j < 3; j++ )
            pos[j] = atof( vals[1 + (i * 3) + j] );
          body.legs[i].goTo( pos, duration, leg_info[i].flags.isset(REVERSED), //leg_info[leg_id].reversed // raw values
                             curtime );
        }
      }
    }
    else if ( !strncmp( cmd, "trot", 4 ) )
    {
      startGait( trot_gait, cmd[4] != 'b' );
    }
    else if ( !strncmp( cmd, "walk", 4 ) )
    {
      startGait( walk_gait, cmd[4] != 'b' );
    }
    else if ( !strcmp( cmd, "rg" ) )
    {
      resetGaits();
    }
    else if ( !strcmp( cmd, "gl" ) ) //gait turn left
    {
      turn_right--;
      setTurn( turn_right, robot_gait );
    }
    else if ( !strcmp( cmd, "gr" ) ) //gait turn right
    {
      turn_right++;
      setTurn( turn_right, robot_gait );
    }
    else if ( !strcmp( cmd, "gs" ) ) //gait go straight
    {
      turn_right = 0;
      shuffle_right = 0;
      setTurn( turn_right, robot_gait );
    }
    else if ( !strcmp( cmd, "gx" ) ) //gait increase X stride
    {
      incrGaitAmplitude( robot_gait, 0, 3 ); //x
    }
    else if ( !strcmp( cmd, "gX" ) ) //gait decrease X stride
    {
      incrGaitAmplitude( robot_gait, 0, -3 ); //x
    }
    else if ( !strcmp( cmd, "gy" ) ) //gait increase Y stride
    {
      incrGaitAmplitude( robot_gait, 1, 3 ); //y
    }
    else if ( !strcmp( cmd, "gY" ) ) //gait decrease Y stride
    {
      incrGaitAmplitude( robot_gait, 1, -3 );
    }
    else if ( !strcmp( cmd, "gd" ) ) //reverse
    {
      gait_direction = false;
    }
    else if ( !strcmp( cmd, "sl" ) ) //gait shuffle left
    {
      shuffle_right--;
      setShuffle( shuffle_right, robot_gait );
    }
    else if ( !strcmp( cmd, "sr" ) ) //gait shuffle right
    {
      shuffle_right++;
      setShuffle( shuffle_right, robot_gait );
    }
    else if ( !strcmp( cmd, "ms" ) ) //manual stepping mode
    {
      body.manual_stepping_mode = !body.manual_stepping_mode;
      body.manual_step_allowed = false;
    }
    else if ( !strcmp( cmd, "mn" ) ) //manual stepping mode next step
    {
      body.manual_step_allowed = true;
    }
    else if ( !strcmp( cmd, "balance" ) ) {
      flags.set(FLAG_IMU);
      robot_gait = NULL;
      body.beginMotion(commands[5].motion); 
    }
    else if ( !strcmp( cmd, "stand" ) )
    {
      //if body is too low, then execute getup instead of stand
      float current_target[3];
      int max_y = 0;
      for ( char i = 0; i < 4; i++ )
      {
        body.legs[i].getPositionXYZ( current_target, true, true );
        if ( current_target[1] > max_y )
          max_y = current_target[1];
        body.beginMotion( max_y < 150 ? &BodyGetup : &BodyStand );
      }
    }
    else if ( cmd[0] == 'm' && cmd[1] == ',' ) //mouth open close
    {
      flags.unset( FLAG_BARK ); //normal
      if ( !strcmp( cmd + 2, "o" ) )
        mouth_target = MOUTH_OPEN;
      else if ( !strcmp( cmd + 2, "f" ) )
        mouth_target = MOUTH_OPEN_MAX;
      else if ( !strcmp( cmd + 2, "c" ) )
        mouth_target = MOUTH_CLOSE;
      else if ( !strcmp( cmd + 2, "b" ) )
      {
        mouth_target = MOUTH_OPEN_MAX;
        flags.set( FLAG_BARK ); //normal
        digitalWrite(BARK_PLAY_E, HIGH);
        delay(30);
        digitalWrite(BARK_PLAY_E, LOW);
      }
      else
      {
        mouth_target = atoi( cmd + 2 );
        if ( mouth_target < MOUTH_CLOSE )
          mouth_target = MOUTH_CLOSE;
        else if ( mouth_target > MOUTH_OPEN_MAX)
          mouth_target = MOUTH_OPEN_MAX;
      }
    }
    else
    {
      for ( char i = 0; i < asize( commands ); i++ )
      {
        if ( !strcmp( commands[i].cmd, cmd ) )
        {
          robot_gait = NULL;
          body.beginMotion( commands[i].motion );
        }
      }
    }
  }
  else
  {
  }
  unsigned long curtime = millis();
  if ( (last_ultrasonic_when + 100) < curtime ) //not more than 10 times a second
  {
    ultrasonic_distance = getUltrasonicDistance( A0, A1 );
    last_ultrasonic_when = curtime;
  }
  if ( !flags.isset( FLAG_PAUSED ) )
  {
    if (flags.isset(FLAG_IMU) && !robot_gait){
      runIMU(false);
      body.beginMotion(commands[5].motion); 
    }
    if ( !body.loop() )
    {
      if ( !Serial.available() ) {
        // runIMU();
         delay( 5 ); //nothing happened, so wait a bit
      }
    }
  }
   
    
  else
  {
    if ( !Serial.available() )
      delay( 10 );
  }
}
