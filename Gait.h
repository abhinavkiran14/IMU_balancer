#ifndef _GAIT_H_
#define _GAIT_H_

struct GaitParam
{
    //moves in ellipse
    int amplitude; //in mm
    int center; //mm
    int phase; //degrees in 360 loop
    char clipmin; //percentage of amplitude, -100 to 100
    char clipmax; //percentage of amplitude, -100 to 100

    float getValue( float deg )
    {
        float val = (float)amplitude* sin( (float)(deg + phase)* DEG_TO_RAD );
        float minmaxval = (float)amplitude * (float)clipmin / 100.0;
        if ( val < minmaxval ) val = minmaxval;
        minmaxval = (float)amplitude * (float)clipmax / 100.0;
        if ( val > minmaxval ) val = minmaxval;
        return (center + val);
    }
};

struct GaitLegParam
{
    GaitParam xyz[3];
    int touchdown_start_deg;
    int touchdown_end_deg;
    int down_duration_ms;
    int up_duration_ms;
    int getTotalDuration() 
    {
      return (down_duration_ms + up_duration_ms);
    }
    //we need to calculate a virtual degree position
    //accounting for different velocities in down and up phases
    int getVirtualDeg( int deg )
    {
      //position in 360 cycle
      //starts with beginning of touchdown
      if ( touchdown_end_deg < touchdown_start_deg )
        touchdown_end_deg += 360;
      int touchdown_deg_range = touchdown_end_deg - touchdown_start_deg;
      int total_ms = down_duration_ms + up_duration_ms;
      float pos = (float)deg / 360.0;
      float ms_into_cycle = pos * (float)total_ms;
            
      if ( ms_into_cycle <= down_duration_ms )
      {
          //scale into deg range of touchdown phase
          deg = (ms_into_cycle / (float)down_duration_ms * (float)touchdown_deg_range) + touchdown_start_deg;
      }
      else //up phase
      {
          //how much into up_phase
          deg = (float)(ms_into_cycle - down_duration_ms)  / 
                  (float)up_duration_ms * (360.0 - touchdown_deg_range) + touchdown_end_deg;
      }
      while( deg > 360 )
        deg -= 360;
      while( deg < 0 )
        deg += 360;

      return deg;
    }
    int getPosition( int deg, float* pos, float speed_factor )
    {
        deg = getVirtualDeg( deg );
        for(char i = 0; i < 3; i++ )
            pos[i] = xyz[i].getValue( deg );
        pos[0] *= speed_factor;
        //not y, since we dont want to increase height of stride with speed, just yet
        pos[2] *= speed_factor; 
        
        return deg;
    }
};

//struct CGOffset
//{
//  char o[3]; //x, y, z, forward/backward, up-down,left-right
//};

struct GaitRobot
{
    GaitLegParam gait;
    int phase_offset;
    //CGOffset* cg_offsets;
    char** cg_offsets;
    char num_cg_offsets;
    char main_offset[3]; //to establish neutral position of this gait/leg
};
/*
int getCGOffset( int deg, CGOffset* cgoffsets, int n )
{
  //n is number of entries in array around 360 loop
  int i = (float)deg / 360.0 * (float)n;
  return i;
}
*/
void setTurn( char turn_right, GaitRobot* robot_gait )
{
  int a = abs( turn_right );
  for(char i = 0; i < 4; i++ )
  {
    robot_gait[i].gait.xyz[2].amplitude = (a *5);
    robot_gait[i].gait.xyz[2].phase = ( 
        ( ( (i == 0 || i == 2) && turn_right <= 0 ) ||
          ( (i == 1 || i == 3) && turn_right >= 0  ) )
                    ? 0 : 180 ); 
  }
  //robot_gait[0].gait.xyz[2].phase = (turn_right <= 0  ? 0 : 180 ); //front
  //robot_gait[2].gait.xyz[2].phase = (turn_right <= 0  ? 0 : 180 ); //front
  //robot_gait[1].gait.xyz[2].phase = (turn_right >=  0 ? 0 : 180 ); //back
  //robot_gait[3].gait.xyz[2].phase = (turn_right >=  0 ? 0 : 180 ); //back
}
void setShuffle( char shuffle_right, GaitRobot* robot_gait )
{
  int a = abs( shuffle_right );
  for(char i = 0; i < 4; i++ )
  {
    robot_gait[i].gait.xyz[2].amplitude = (a *5);
    robot_gait[i].gait.xyz[2].phase = ( 
        ( ( (i == 0 || i == 3) && shuffle_right <= 0 ) ||
          ( (i == 1 || i == 2) && shuffle_right >= 0  ) )
                    ? 0 : 180 ); 
  }
            
  //robot_gait[0].gait.xyz[2].phase = (shuffle_right <= 0  ? 0 : 180 ); //front left
  //robot_gait[1].gait.xyz[2].phase = (shuffle_right >= 0  ? 0 : 180 ); //back left
  //robot_gait[2].gait.xyz[2].phase = (shuffle_right >=  0 ? 0 : 180 ); //front right
  //robot_gait[3].gait.xyz[2].phase = (shuffle_right <=  0 ? 0 : 180 ); //back right
}
void incrGaitAmplitude( GaitRobot* robot_gait, char which, char howmuch )
{
  //dont let it get negative
  for(char i = 0; i < 4; i++ )
      robot_gait[i].gait.xyz[which].amplitude = abs(robot_gait[i].gait.xyz[which].amplitude + howmuch);
}

#endif //_GAIT_H_
