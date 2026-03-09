//
//	MBsysTran - Release 8.1
//
//	Copyright 
//	Universite catholique de Louvain (UCLouvain) 
//	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
//	2, Place du Levant
//	1348 Louvain-la-Neuve 
//	Belgium 
//
//	http://www.robotran.be 
//
//	==> Generation Date: Tue Nov 18 11:18:53 2025
//	==> using automatic loading with extension .mbs 
//
//	==> Project name: car
//
//	==> Number of joints: 43
//
//	==> Function: F7 - Link Forces (1D)
//
//	==> Git hash: fe15d6b972a9ca9ecad0489d82703366c2afa3a1
//
////

#include <math.h> 

#include "mbs_data.h"

#include "mbs_project_interface.h"

void mbs_link(double **frc, double **trq, double *Flink, double *Z, double *Zd,
MbsData *s, double tsim)
{
#include "mbs_link_car.h"

double *q, *qd;
double **l, **dpt;

frc = s->frc;
trq = s->trq;
Z = s->Z;
Zd = s->Zd;

q = s->q;
qd = s->qd;

dpt = s->dpt;
l  = s->l;
 
// Trigonometric functions

S17 = sin(q[17]);
C17 = cos(q[17]);
S18 = sin(q[18]);
C18 = cos(q[18]);
S19 = sin(q[19]);
C19 = cos(q[19]);
S20 = sin(q[20]);
C20 = cos(q[20]);
 
// Augmented Joint Position Vectors

 
// Link anchor points Kinematics

RLlnk2_22 = dpt[2][29]*C17;
RLlnk2_32 = dpt[2][29]*S17;
POlnk2_22 = RLlnk2_22+dpt[2][3];
ORlnk2_22 = -qd[17]*RLlnk2_32;
ORlnk2_32 = qd[17]*RLlnk2_22;
Plnk21 = POlnk2_22-dpt[2][11];
Plnk31 = RLlnk2_32-dpt[3][11];
PPlnk1 = Plnk21*Plnk21+Plnk31*Plnk31;
Z1 = sqrt(PPlnk1);
e21 = Plnk21/Z1;
e31 = Plnk31/Z1;
Zd1 = ORlnk2_22*e21+ORlnk2_32*e31;
RLlnk4_22 = dpt[2][31]*C18;
RLlnk4_32 = dpt[2][31]*S18;
POlnk4_22 = RLlnk4_22+dpt[2][4];
ORlnk4_22 = -qd[18]*RLlnk4_32;
ORlnk4_32 = qd[18]*RLlnk4_22;
Plnk22 = POlnk4_22-dpt[2][14];
Plnk32 = RLlnk4_32-dpt[3][14];
PPlnk2 = Plnk22*Plnk22+Plnk32*Plnk32;
Z2 = sqrt(PPlnk2);
e22 = Plnk22/Z2;
e32 = Plnk32/Z2;
Zd2 = ORlnk4_22*e22+ORlnk4_32*e32;
RLlnk6_22 = dpt[2][33]*C19;
RLlnk6_32 = dpt[2][33]*S19;
POlnk6_22 = RLlnk6_22+dpt[2][5];
ORlnk6_22 = -qd[19]*RLlnk6_32;
ORlnk6_32 = qd[19]*RLlnk6_22;
Plnk13 = dpt[1][5]-dpt[1][6];
Plnk23 = POlnk6_22-dpt[2][6];
Plnk33 = RLlnk6_32-dpt[3][6];
PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33;
Z3 = sqrt(PPlnk3);
e13 = Plnk13/Z3;
e23 = Plnk23/Z3;
e33 = Plnk33/Z3;
Zd3 = ORlnk6_22*e23+ORlnk6_32*e33;
RLlnk7_22 = dpt[2][35]*C20;
RLlnk7_32 = dpt[2][35]*S20;
POlnk7_22 = RLlnk7_22+dpt[2][7];
ORlnk7_22 = -qd[20]*RLlnk7_32;
ORlnk7_32 = qd[20]*RLlnk7_22;
Plnk14 = dpt[1][10]-dpt[1][7];
Plnk24 = -POlnk7_22+dpt[2][10];
Plnk34 = -RLlnk7_32+dpt[3][10];
PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34;
Z4 = sqrt(PPlnk4);
e14 = Plnk14/Z4;
e24 = Plnk24/Z4;
e34 = Plnk34/Z4;
Zd4 = -ORlnk7_22*e24-ORlnk7_32*e34;

// Link Forces 

Flink1 = user_call_LinkForces(Z1,Zd1,s,tsim,1);
Flink2 = user_call_LinkForces(Z2,Zd2,s,tsim,2);
Flink3 = user_call_LinkForces(Z3,Zd3,s,tsim,3);
Flink4 = user_call_LinkForces(Z4,Zd4,s,tsim,4);
 
// Link Dynamics: forces projection on body-fixed frames

fPlnk21 = Flink1*e21;
fPlnk31 = Flink1*e31;
trqlnk6_1_1 = -fPlnk21*(dpt[3][11]-l[3][6])+fPlnk31*dpt[2][11];
trqlnk6_1_2 = fPlnk31*l[1][6];
trqlnk6_1_3 = -fPlnk21*l[1][6];
fSlnk21 = Flink1*(e21*C17+e31*S17);
fSlnk31 = Flink1*(-e21*S17+e31*C17);
trqlnk17_1_1 = -fSlnk31*dpt[2][29];
fPlnk22 = Flink2*e22;
fPlnk32 = Flink2*e32;
frclnk6_2_2 = fPlnk21+fPlnk22;
frclnk6_2_3 = fPlnk31+fPlnk32;
trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*(dpt[3][14]-l[3][6])+fPlnk32*dpt[2][14];
trqlnk6_2_2 = trqlnk6_1_2+fPlnk32*l[1][6];
trqlnk6_2_3 = trqlnk6_1_3-fPlnk22*l[1][6];
fSlnk22 = Flink2*(e22*C18+e32*S18);
fSlnk32 = Flink2*(-e22*S18+e32*C18);
trqlnk18_2_1 = -fSlnk32*dpt[2][31];
fPlnk13 = Flink3*e13;
fPlnk23 = Flink3*e23;
fPlnk33 = Flink3*e33;
frclnk6_3_2 = fPlnk23+frclnk6_2_2;
frclnk6_3_3 = fPlnk33+frclnk6_2_3;
trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*(dpt[3][6]-l[3][6])+fPlnk33*dpt[2][6];
trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*(dpt[3][6]-l[3][6])-fPlnk33*(dpt[1][6]-l[1][6]);
trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*dpt[2][6]+fPlnk23*(dpt[1][6]-l[1][6]);
fSlnk13 = Flink3*e13;
fSlnk23 = Flink3*(e23*C19+e33*S19);
fSlnk33 = Flink3*(-e23*S19+e33*C19);
trqlnk19_3_1 = -fSlnk33*dpt[2][33];
trqlnk19_3_3 = fSlnk13*dpt[2][33];
fPlnk14 = Flink4*e14;
fPlnk24 = Flink4*(e24*C20+e34*S20);
fPlnk34 = Flink4*(-e24*S20+e34*C20);
trqlnk20_4_1 = fPlnk34*dpt[2][35];
trqlnk20_4_3 = -fPlnk14*dpt[2][35];
fSlnk14 = Flink4*e14;
fSlnk24 = Flink4*e24;
fSlnk34 = Flink4*e34;
frclnk6_4_1 = fPlnk13-fSlnk14;
frclnk6_4_2 = -fSlnk24+frclnk6_3_2;
frclnk6_4_3 = -fSlnk34+frclnk6_3_3;
trqlnk6_4_1 = trqlnk6_3_1+fSlnk24*(dpt[3][10]-l[3][6])-fSlnk34*dpt[2][10];
trqlnk6_4_2 = trqlnk6_3_2-fSlnk14*(dpt[3][10]-l[3][6])+fSlnk34*(dpt[1][10]-l[1][6]);
trqlnk6_4_3 = trqlnk6_3_3+fSlnk14*dpt[2][10]-fSlnk24*(dpt[1][10]-l[1][6]);
 
// Symbolic model output

frc[1][6] = frc[1][6]+frclnk6_4_1;
frc[2][6] = frc[2][6]+frclnk6_4_2;
frc[3][6] = frc[3][6]+frclnk6_4_3;
trq[1][6] = trq[1][6]+trqlnk6_4_1;
trq[2][6] = trq[2][6]+trqlnk6_4_2;
trq[3][6] = trq[3][6]+trqlnk6_4_3;
frc[2][17] = frc[2][17]-fSlnk21;
frc[3][17] = frc[3][17]-fSlnk31;
trq[1][17] = trq[1][17]+trqlnk17_1_1;
frc[2][18] = frc[2][18]-fSlnk22;
frc[3][18] = frc[3][18]-fSlnk32;
trq[1][18] = trq[1][18]+trqlnk18_2_1;
frc[1][19] = frc[1][19]-fSlnk13;
frc[2][19] = frc[2][19]-fSlnk23;
frc[3][19] = frc[3][19]-fSlnk33;
trq[1][19] = trq[1][19]+trqlnk19_3_1;
trq[3][19] = trq[3][19]+trqlnk19_3_3;
frc[1][20] = frc[1][20]+fPlnk14;
frc[2][20] = frc[2][20]+fPlnk24;
frc[3][20] = frc[3][20]+fPlnk34;
trq[1][20] = trq[1][20]+trqlnk20_4_1;
trq[3][20] = trq[3][20]+trqlnk20_4_3;
 
// Symbolic model output

Z[1] = Z1;
Zd[1] = Zd1;
Flink[1] = Flink1;
Z[2] = Z2;
Zd[2] = Zd2;
Flink[2] = Flink2;
Z[3] = Z3;
Zd[3] = Zd3;
Flink[3] = Flink3;
Z[4] = Z4;
Zd[4] = Zd4;
Flink[4] = Flink4;

// Number of continuation lines = 0

}
