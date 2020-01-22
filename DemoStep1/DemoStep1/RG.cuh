#ifndef __RGKERNEL_H
#define __RGKERNEL_H

#include "cudaTypes.cuh"

///**** Function definitions ****/
vector<float *> DetectPlans_cu(float *VMap, float *NMap, int *Indices_Final_dev, float epsilon, float alpha, int n, int m);

void Project_on_primitives_cu(float *Projected_dev, int *BBox, float *VMap, float *NMap, float *RGB, int *Indices_Final_dev, float *Equations_dev, int nbPlans, float res, float epsilon, float alpha, int n, int m);

void Segment_cu(unsigned char *Label, float *VMap, float *NMap, float *pose, float *Equations, int nbPlans, float epsilon, float alpha, int n, int m);

void InitFrags_cu(float *Projected_dev, int *Indices, int *Size, float *center, unsigned short *TheBumps_dev, unsigned char *TheRGBs_dev, unsigned char *TheMasks_dev, float *equation, int j, float res, int n, int m);

void init_val(unsigned char *InOutput, unsigned char val, int n, int m);

bool *ImDilate(bool *Im, int n, int m, int size);

bool *ImDilate(unsigned char *Im, int n, int m, int size);

bool *ImErode(bool *Im, int n, int m, int size);

void VotePlan_cu(unsigned char *Label, float *VMap, float *centers, float *count, float *pose, int n, int m);

void AffectPlan_cu(unsigned char *Label, int *Buff, int n, int m);

void Affect(float **Tab, float *in, int indx);
void AffectChar(unsigned char **Tab, unsigned char *in, int indx);
void AffectShort(unsigned short **Tab, unsigned short *in, int indx);

#endif