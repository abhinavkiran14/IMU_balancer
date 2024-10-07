#ifndef _UTILS_H_
#define _UTILS_H_

#define asize(A) (sizeof(A) / sizeof(A[0]))

double getangleabc( double ab, double bc, double ca )
{
    double v = (ab*ab + bc*bc - ca*ca) / (2.0 * ab * bc );
    if ( v > 1 )
        v = 1;
    if ( v < -1 )
        v = -1;
    return (acos(v) * RAD_TO_DEG);
}

int floatprintf3( char* buf, int sz, float val )
{
  int l = (int)val;
  float r = val - (float)l;
  if ( r == 0 )
    return snprintf( buf, sz, "%d", l );

  if ( r < 0 )
    r = -r;
    
  return snprintf( buf, sz, "%d.%03d", l, round(r*1000.0) );
}
int split(char *line, char sep, int mxfld, char** fields )
{
    char *w  = line;
    int  cnt = 0;

    while ( *line ) {
        if ( *line == sep ) {
            if ( cnt >= mxfld )
                return cnt;
            *line++ = '\0';
            fields[cnt++] = w;
            w = line;
        } else {
            line++;
        }
    }
    if ( line >= w && cnt < mxfld )
        fields[cnt++] = w;
    return cnt;
}

bool isValidAngle( float ang )
{
  return ( ang >= -361 && ang <= 361 );
}

#endif //_UTILS_H_
