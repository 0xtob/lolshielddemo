/*
 * tgl.cpp
 *
 *  Created on: Dec 29, 2010
 *      Author: tob
 */

#include "tgl.h"
#include <Charliplexing.h>
#include "settings.h"
#include <WProgram.h>
/*
MatrixStack stack_modelview;
MatrixStack stack_projection;
*/
float m_modelview[16];
float m_projection[16];
float m_window[16];
float m_tmp[16];

int matrix_mode = TGL_MODELVIEW;

int point_size = 1;

int clear_color[] = {0, 0, 0};
int draw_mode = TGL_POINTS;
int drawing = 0;
int tgl_lines_is_firstvtx = 0;
float tgl_lines_prev_vtx[4];

unsigned char display[14][9];

// ========================= PUBLIC FUNCTIONS ======================
void PxlOnLOL(int x, int y)
{
  if( (x< 0) || (x >= SCREEN_WIDTH) || (y <  0) || (y >= SCREEN_HEIGHT) )
    return;

  display[x][y] = 1;
}

void tglInit()
{
  //tglUtilStackInit(&stack_modelview);
  //tglUtilStackInit(&stack_projection);
  tglClearColor(0.0, 0.0, 0.0);
  tglMatrixMode(TGL_PROJECTION);
  tglLoadIdentity();
  tglMatrixMode(TGL_MODELVIEW);
  tglLoadIdentity();
}

void tglViewport(int x, int y, int w, int h)
{
  for(int i=0; i<16; ++i) m_window[i] = 0;
  m_window[0] = (float)w / 2.0;
  m_window[3] = (float)w / 2.0 + (float)x;
  m_window[5] = (float)h / 2.0;
  m_window[7] = (float)h / 2.0 + (float)y;
  m_window[15] = 1.0;
}

void tglMatrixMode(int matrix_mode_)
{
  matrix_mode = matrix_mode_;
}

void tglLoadIdentity()
{
  float *m_current = tglUtilGetCurrentMatrix();

  m_current[0] = 1.0; m_current[1] = 0.0; m_current[2] = 0.0; m_current[3] = 0.0;
  m_current[4] = 0.0; m_current[5] = 1.0; m_current[6] = 0.0; m_current[7] = 0.0;
  m_current[8] = 0.0; m_current[9] = 0.0; m_current[10] = 1.0; m_current[11] = 0.0;
  m_current[12] = 0.0; m_current[13] = 0.0; m_current[14] = 0.0; m_current[15] = 1.0;
}

void tglMultMatrixf(float *matrix)
{
  float *res = tglUtilGetCurrentMatrix();
  //float m_current[16];
  for(int i=0; i<16; ++i) m_tmp[i] = res[i];

  for(int x=0; x<4; ++x) {
    for(int y=0; y<4; ++y) {
      res[4 * y + x] = 0;
      for(int z=0; z<4; ++z) {
        res[4 * y + x] += m_tmp[y * 4 + z] * matrix[z * 4 + x];
      }
    }
  }
}

void tglFrustum(float left, float right, float bottom, float top, float near, float far)
{
  float frustum_matrix[] =
  { -2.0 * near / (right - left),                             0,  (left + right) / (right - left),                                 0,
                               0,  -2.0 * near / (top - bottom),  (top + bottom) / (top - bottom),                                 0,
                               0,                             0,      (near + far) / (near - far),  -2.0 * near * far / (near - far),
                               0,                             0,                               -1,                                 0 };

  tglMultMatrixf(frustum_matrix);
}

void tglTranslatef(float x, float y, float z)
{
  float trans_matrix[] = { 1, 0, 0, x,
                           0, 1, 0, y,
                           0, 0, 1, z,
                           0, 0, 0, 1};

  tglMultMatrixf(trans_matrix);
}

void tglScalef(float x, float y, float z)
{
  float scale_matrix[] = { x, 0, 0, 0,
                           0, y, 0, 0,
                           0, 0, z, 0,
                           0, 0, 0, 1};

  tglMultMatrixf(scale_matrix);
}

void tglRotatef(float angle, float x, float y, float z)
{
  float c = cos(angle*M_PI/180.0);
  float s = sin(angle*M_PI/180.0);
  float ci = 1 - c;

  float vec[] = {x, y, z};
  tglUtilNormalize3f(vec, vec);
  float nx = vec[0];
  float ny = vec[1];
  float nz = vec[2];

  float rot_matrix[] = {    nx*nx*ci + c,       nx*ny*ci - nz*s, nx*nz*ci + ny*s, 0,
                            nx*ny*ci + nz*s,    ny*ny*ci + c,    ny*nz*ci - nx*s, 0,
                            nx*nz*ci - ny*s,    ny*nz*ci + nx*s, nz*nz*ci + c   , 0,
                                          0,                  0,               0, 1 };

  tglMultMatrixf(rot_matrix);
}
/*
void tglPushMatrix()
{
  switch(matrix_mode)
  {
    case TGL_MODELVIEW:
      tglUtilStackPush(&stack_modelview, m_modelview);
      break;

    case TGL_PROJECTION:
      tglUtilStackPush(&stack_projection, m_projection);
      break;
  }
}

void tglPopMatrix()
{
  switch(matrix_mode)
  {
    case TGL_MODELVIEW:
      tglUtilStackPop(&stack_modelview, m_modelview);
      break;

    case TGL_PROJECTION:
      tglUtilStackPop(&stack_projection, m_projection);
      break;
  }
}
*/
void tglBegin(int draw_mode_)
{
  draw_mode = draw_mode_;
  drawing = 1;

  switch(draw_mode) {
    case TGL_LINES:
      tgl_lines_is_firstvtx = 1;
      break;
  }
}

void tglEnd()
{
  drawing = 0;
}

void tglVertex3f(float x, float y, float z)
{
  float vec[] = {x, y, z, 1};
  tglVertex4fv(vec);
}

void tglVertex3fv(float *vec)
{
  float vec4[] = {vec[0], vec[1], vec[2], 1};
  tglVertex4fv(vec4);
}

void tglVertex4fv(float *vec)
{
  if(drawing == 0)
    return;

  //println("vec: "+vec[0]+" "+vec[1]+" "+vec[2]+" "+vec[3]+" ");

  // Modelview Transform
  tglUtilMultMatrixVector(m_modelview, vec, vec);

  //println("model -> "+vec[0]+" "+vec[1]+" "+vec[2]+" "+vec[3]+" ");

  // Projection
  tglUtilMultMatrixVector(m_projection, vec, vec);

  //printf("proj -> [%f, %f, %f, %f]\n", vec[0], vec[1], vec[2], vec[3]);

  // Map to window coordinates
  tglUtilMultMatrixVector(m_window, vec, vec);

  //printf("win -> [%f, %f, %f, %f]\n", vec[0], vec[1], vec[2], vec[3]);

  // Dehomogenize
  tglUtilDehomogenize(vec, vec);

  //printf("dehomo -> [%f, %f, %f, %f]\n", vec[0], vec[1], vec[2], vec[3]);

  // Draw
  switch(draw_mode)
  {
    case(TGL_POINTS):
    {
      //ellipse((int)round(vec[0]), (int)round(vec[1]), point_size, point_size);
      PxlOnLOL(round(vec[0]), round(vec[1]));
      break;
    }

    case(TGL_LINES):
    {
      if(tgl_lines_is_firstvtx) {
        tgl_lines_is_firstvtx = 0;
      } else {
        tglUtilDrawBresLine(tgl_lines_prev_vtx, vec);
      }
      tglUtilCopyVtx4f(tgl_lines_prev_vtx, vec);
    }
  }
}


void tglClear(int bitmask)
{
  if( (bitmask & TGL_COLOR_BUFFER_BIT) != 0) {
    for(int j=0;j<SCREEN_HEIGHT;++j)
      for(int i=0;i<SCREEN_WIDTH;++i)
        //LedSign::Set(i,j,0);
        display[i][j] = 0;
  }
}

void tglClearColor(float r, float g, float b)
{
  clear_color[0] = (int)floor(r * 255.0);
  clear_color[1] = (int)floor(g * 255.0);
  clear_color[2] = (int)floor(b * 255.0);
}

void tglSwap()
{
  for(int j=0;j<9;++j)
    for(int i=0;i<14;++i)
      LedSign::Set(i,j,display[i][j]);
  //LedSign::Flip(true);
}

// ========================= UTILITY FUNCTIONS ====================== //

float *tglUtilMultMatrixVector(float *matrix, float *vec, float *res)
{
  // Copy the input vector in case vec == res
  float vec_t[4];
  for(int i=0; i<4; ++i) vec_t[i] = vec[i];

  for(int row=0; row<4; ++row) {
    res[row] = 0;
    for(int col=0; col<4; ++col) {
      res[row] += matrix[row * 4 + col] * vec_t[col];
    }
  }

  return res;
}

float *tglUtilDehomogenize(float *vec, float *res)
{
  for(int col=0; col<4; ++col) {
    res[col] = vec[col] / vec[3];
  }

  return res;
}

void tglUtilPrintMatrix(float *matrix)
{
  for(int row=0; row<4; ++row) {
    for(int col=0; col<4; ++col) {
      //printf("%f ", matrix[row * 4 + col]);
    }
    //printf("\n");
  }
}

float *tglUtilGetCurrentMatrix()
{
  switch(matrix_mode)
    {
      case TGL_MODELVIEW:
        return m_modelview;

      case TGL_PROJECTION:
        return m_projection;

      default:
        return m_modelview;
    }
}

void tglUtilSetCurrentMatrix(float *matrix)
{
  switch(matrix_mode)
    {
      case TGL_MODELVIEW:
        tglUtilCopyMatrix(m_modelview, matrix);
        break;

      case TGL_PROJECTION:
        tglUtilCopyMatrix(m_projection, matrix);
        break;
    }
}

void tglUtilCopyMatrix(float *dest, float* src) {
    for(int i=0; i<16; ++i)
        dest[i] = src[i];
}

float *tglUtilNormalize3f(float *vec, float *res)
{
  float len = sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
  res[0] = vec[0] / len;
  res[1] = vec[1] / len;
  res[2] = vec[2] / len;
  return res;
}
/*
void tglUtilStackPush(MatrixStack *stack, float *matrix) {
  if(stack->size == M_STACK_MAX_SIZE)
    return;

  tglUtilCopyMatrix(stack->m[stack->size], matrix);

  stack->size++;
}

void tglUtilStackPop(MatrixStack *stack, float *res) {
  if(stack->size == 0) {
    res = 0;
    return;
  }
  tglUtilCopyMatrix(res, stack->m[stack->size - 1]);

  stack->size--;
}

void tglUtilStackInit(MatrixStack *stack)
{
    stack->size = 0;
}
*/
void tglUtilDrawBresLine(float *from, float *to)
{
  int x1 = (int)round(from[0]);
  int y1 = (int)round(from[1]);
  int x2 = (int)round(to[0]);
  int y2 = (int)round(to[1]);

  // Guarantees that all lines go from left to right
  if ( x2 < x1 ) {
  	int tmp;
  	tmp = x2; x2 = x1; x1 = tmp;
  	tmp = y2; y2 = y1; y1 = tmp;
  }

  int dy,dx;
  dy = y2 - y1;
  dx = x2 - x1;

  // If the gradient is greater than one we have to flip the axes
  if ( abs(dy) < dx ) {
  	int xp,yp;
  	int d;
  	int add = 1;

  	xp=x1;
  	yp=y1;

  	if(dy < 0) {
  		dy = -dy;
  		add=-1;
  	}

  	d = 2*dy - dx;

  	for(; xp<=x2; xp++) {

  		if(d>0) {
  			yp+=add;
  			d -= 2*dx;
  		}
  		//printf("x: %d, y: %d\n", xp, yp);

  		PxlOnLOL(xp, yp);

  		d += 2*dy;
  	}

  } else {

  	int tmp;
  	tmp = x1; x1 = y1; y1 = tmp;
  	tmp = x2; x2 = y2; y2 = tmp;

  	if ( x2 < x1 ) {
  		tmp = x2; x2 = x1; x1 = tmp;
  		tmp = y2; y2 = y1; y1 = tmp;
  	}

  	int xp,yp;
  	int d;

  	dy = y2 - y1;
  	dx = x2 - x1;

  	int add = 1;
  	if(dy < 0) {
  		dy = -dy;
  		add=-1;
  	}

  	xp=x1;
  	yp=y1;

  	d = 2*dy - dx;

  	for(xp=x1; xp<=x2; xp++) {

  		if(d>0) {
  			yp+=add;
  			d -= 2*dx;
  		}

  		PxlOnLOL(yp, xp);


  		//printf("x: %d, y: %d, d: %d\n", xp, yp, d);
  		d += 2*dy;
  	}
  }
}

void tglUtilCopyVtx4f(float *dest, float *src)
{
    for(int i=0; i<4; ++i) dest[i] = src[i];
}
