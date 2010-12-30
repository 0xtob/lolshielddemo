/*
 * tgl.h
 *
 *  Created on: Dec 29, 2010
 *      Author: 0xtob
 */

#ifndef TGL_H_
#define TGL_H_

/* Tob's pointless OpenGL library. */

#include <math.h>

// Matrix Modes
#define TGL_MODELVIEW 0
#define TGL_PROJECTION 1

// Draw Modes
#define TGL_POINTS 0
#define TGL_POLYGONS 1
#define TGL_LINES 2

// Buffer bits
#define TGL_COLOR_BUFFER_BIT 1

// ========================= PUBLIC FUNCTIONS ====================== //

void tglInit();
void tglViewport(int x, int y, int w, int h);
void tglMatrixMode(int matrix_mode_);
void tglLoadIdentity();
void tglMultMatrixf(float *matrix);
void tglFrustum(float left, float right, float bottom, float top, float near, float far);
void tglTranslatef(float x, float y, float z);
void tglScalef(float x, float y, float z);
void tglRotatef(float angle, float x, float y, float z);
void tglPushMatrix();
void tglPopMatrix();
void tglBegin(int draw_mode_);
void tglEnd();
void tglVertex3f(float x, float y, float z);
void tglVertex3fv(float *vec);
void tglVertex4fv(float *vec);
void tglClear(int bitmask);
void tglClearColor(float r, float g, float b);
void tglSwap();

// ========================= UTILITY FUNCTIONS ====================== //

float *tglUtilMultMatrixVector(float *matrix, float *vec, float *res);
float *tglUtilDehomogenize(float *vec, float *res);
void tglUtilPrintMatrix(float *matrix);
float *tglUtilGetCurrentMatrix();
void tglUtilSetCurrentMatrix(float *matrix);
void tglUtilCopyMatrix(float *dest, float* src);
float *tglUtilNormalize3f(float *vec, float *res);
void tglUtilDrawBresLine(float *from, float *to);
void tglUtilCopyVtx4f(float *dest, float *src);

#endif /* TGL_H_ */
