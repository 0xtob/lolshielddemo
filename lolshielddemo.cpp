#include <WProgram.h>
#include <Charliplexing.h>
#include "tgl.h"
#include "settings.h"

int frame = 0;
int effect = 1;
int i = 0;

extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

float cube[16][4] =  {
                      {-1, -1, -1},
                      { 1, -1, -1},
                      { 1,  1, -1},
                      {-1,  1, -1},
                      {-1, -1,  1},
                      { 1, -1,  1},
                      { 1,  1,  1},
                      {-1,  1,  1}
                    };

int cubestrip1[] = {7,4,5,6,7,3,2,6};
int cubestrip2[] = {0,1,2,3,0,4,5,1};

#define N_SCENES 3
unsigned int timeline[] = {5000, 2500, 15000};
unsigned int timeline_cumsum[N_SCENES];
int scene = 0;
int scene_time = 0;

void initGL()
{
  tglInit();
  tglViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

  tglMatrixMode(TGL_PROJECTION);
  float aspect = (float)SCREEN_HEIGHT / (float)SCREEN_WIDTH;
  tglFrustum(-1.0, 1.0, -aspect, aspect, 1.0, 5.0);
}

void setup()
{
//  digitalWrite(13,1);
  LedSign::Init();
  initGL();

  unsigned int cumsum = 0;
  for (int var = 0; var < N_SCENES; ++var) {
    cumsum += timeline[var];
    timeline_cumsum[var] = cumsum;
  }
}

// DNA
inline void scene0(float scene_time) {
  float offset = 3.14/2.0;

  for(int i=0;i<14;++i) {
    int high = (int)((1.0+sin((float)(i+frame)/2))*4);
    LedSign::Set(i,high,0);
  }

  for(int i=0;i<14;++i) {
    int high = (int)((1.0+sin(offset+(float)(i+frame)/2))*4);
    LedSign::Set(i,high,0);
  }

  frame++;

  for(int i=0;i<14;++i) {
    int high = (int)((1.0+sin((float)(i+frame)/2))*4);
    LedSign::Set(i,high,1);
  }

  for(int i=0;i<14;++i) {
    int high = (int)((1.0+sin(offset+(float)(i+frame)/2))*4);
    LedSign::Set(i,high,1);
  }

  delay(40);
}

// Zoom into the cube
inline void scene1(float scene_time)
{
  tglClear(TGL_COLOR_BUFFER_BIT);  

  //float zoom = 0.5*sin((float)i/10.0);
  float min_zoom = -10;
  float max_zoom = -2.5;
  float zoom = min_zoom + (max_zoom-min_zoom) * (float)scene_time/timeline[scene];

  tglMatrixMode(TGL_MODELVIEW);
  tglLoadIdentity();
  tglTranslatef(0, 0, zoom);

  //tglRotatef(i % 360, 0, 1, 0);
  //tglRotatef(i % 360, 1, 1, 0);

  tglBegin(TGL_LINES);
  for(int p=0; p<8; ++p) {
    tglVertex3fv(cube[cubestrip1[p]]);
  }
  tglEnd();

  tglBegin(TGL_LINES);
  for(int p=0; p<8; ++p) {
    tglVertex3fv(cube[cubestrip2[p]]);
  }
  tglEnd();

  tglSwap();
  
  delay(50);
}

inline void scene2(float scene_time)
{
  int i = scene_time / 25;

  tglClear(TGL_COLOR_BUFFER_BIT);

  float zoom = 0;
  if(scene_time > 5000)
    zoom = 0.5*sin((float)(i-20)/10.0);
    
  float rot_speed = 1.0;
  if(scene_time > 12000)
    rot_speed += (float)(scene_time - 12000) / 1500.0;

  tglMatrixMode(TGL_MODELVIEW);
  tglLoadIdentity();
  tglTranslatef(0, 0, -2.5+zoom);

  tglRotatef((float)(i % 360) * rot_speed, 0, 1, 0);
  tglRotatef((float)(i % 360) * rot_speed, 1, 1, 0);

  tglBegin(TGL_LINES);
  for(int p=0; p<8; ++p) {
    tglVertex3fv(cube[cubestrip1[p]]);
  }
  tglEnd();

  tglBegin(TGL_LINES);
  for(int p=0; p<8; ++p) {
    tglVertex3fv(cube[cubestrip2[p]]);
  }
  tglEnd();

  tglSwap();

  //delay(50);
}

void loop()
{
  //LedSign::Set(1,1,1);
  //delay(100);
  //return; 
  
  unsigned int abstime = millis() % timeline_cumsum[N_SCENES-1];

  for (int var = 0; var < N_SCENES; ++var) {
    if(timeline_cumsum[var] > abstime) {
      scene = var;
      break;
    }
  }

  if(scene == 0)
    scene_time = abstime;
  else
    scene_time = abstime - timeline_cumsum[scene-1];

/*
  // Debug: Show Scene
  LedSign::Set(0,0,0);
  LedSign::Set(1,0,0);
  LedSign::Set(2,0,0);
  LedSign::Set(scene,0,1);
*/
  switch(scene) {
    case 0: {
      scene0(scene_time);
      break;
    }

    case 1: {
      scene1(scene_time);
      break;
    }

    case 2: {
      scene2(scene_time);
      break;
    }
  }
 
  /*
  for(int j=0; j<9; ++j) {
    for(int i=0; i<14; ++i) {
      LedSign::Set(i, j, (i+j+(int)(5.0*(1.0+sin((float)frame/5.0))))%5);
    }
  }*/
  //delay(40);
  
//  frame++;
}

int main(void)
{
    init();	   // initialize hardware and data structures for runtime library

    setup();	  // call user-defined setup routine

    for (;;)
	  loop();     // keep calling user-defined loop routine forever

    return 0;	 // we never get here, but make the compiler happy about main returning int
}
