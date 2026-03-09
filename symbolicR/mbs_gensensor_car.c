//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Wed Oct  3 12:17:36 2018
//
//	==> Project name : car
//	==> using XML input file 
//
//	==> Number of joints : 43
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 9369
//
//	==> Generation Time :  0.140 seconds
//	==> Post-Processing :  0.180 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "mbs_sensor.h"
 
void  mbs_gensensor(MbsSensor *sens, 
              MbsData *s,
              int isens)
{ 
 
#include "mbs_gensensor_car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

// = = Block_0_0_0_0_0_2 = = 
 
// Trigonometric Variables  

  C7 = cos(q[7]);
  S7 = sin(q[7]);
  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C12 = cos(q[12]);
  S12 = sin(q[12]);
  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C17 = cos(q[17]);
  S17 = sin(q[17]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);
  C23 = cos(q[23]);
  S23 = sin(q[23]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C24 = cos(q[24]);
  S24 = sin(q[24]);
  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);

// = = Block_0_0_0_0_0_12 = = 
 
// Trigonometric Variables  

  C28 = cos(q[28]);
  S28 = sin(q[28]);
  C29 = cos(q[29]);
  S29 = sin(q[29]);

// = = Block_0_0_0_0_0_13 = = 
 
// Trigonometric Variables  

  C30 = cos(q[30]);
  S30 = sin(q[30]);
  C31 = cos(q[31]);
  S31 = sin(q[31]);

// = = Block_0_0_0_0_0_14 = = 
 
// Trigonometric Variables  

  C32 = cos(q[32]);
  S32 = sin(q[32]);
  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);

// = = Block_0_0_0_0_0_15 = = 
 
// Trigonometric Variables  

  C35 = cos(q[35]);
  S35 = sin(q[35]);
  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C38 = cos(q[38]);
  S38 = sin(q[38]);

// = = Block_0_0_0_0_0_17 = = 
 
// Trigonometric Variables  

  C39 = cos(q[39]);
  S39 = sin(q[39]);
  C40 = cos(q[40]);
  S40 = sin(q[40]);
  C41 = cos(q[41]);
  S41 = sin(q[41]);

// = = Block_0_0_0_0_0_18 = = 
 
// Trigonometric Variables  

  C42 = cos(q[42]);
  S42 = sin(q[42]);
  C43 = cos(q[43]);
  S43 = sin(q[43]);

// = = Block_0_0_0_21_0_8 = = 
 
// Trigonometric Variables  

  S21p6 = C21*S6+S21*C6;
  C21p6 = C21*C6-S21*S6;

// = = Block_0_0_0_24_0_10 = = 
 
// Trigonometric Variables  

  S21p6p24 = C21p6*S24+S21p6*C24;
  C21p6p24 = C21p6*C24-S21p6*S24;

// = = Block_0_0_0_38_0_16 = = 
 
// Trigonometric Variables  

  S38p6 = C38*S6+S38*C6;
  C38p6 = C38*C6-S38*S6;

// = = Block_0_0_0_39_0_17 = = 
 
// Trigonometric Variables  

  S38p6p39 = C38p6*S39+S38p6*C39;
  C38p6p39 = C38p6*C39-S38p6*S39;

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = (1.0);
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->A[1] = qdd[1];
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = (1.0);
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = (1.0);
    sens->R[2][2] = (1.0);
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = C4;
    sens->R[1][2] = S4;
    sens->R[2][1] = -S4;
    sens->R[2][2] = C4;
    sens->R[3][3] = (1.0);
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[3] = qd[4];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
    sens->OMP[3] = qdd[4];
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_45 = -S4*C5;
    ROcp4_55 = C4*C5;
    ROcp4_75 = S4*S5;
    ROcp4_85 = -C4*S5;
    OMcp4_15 = qd[5]*C4;
    OMcp4_25 = qd[5]*S4;
    OPcp4_15 = qdd[5]*C4-qd[4]*qd[5]*S4;
    OPcp4_25 = qdd[5]*S4+qd[4]*qd[5]*C4;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = C4;
    sens->R[1][2] = S4;
    sens->R[2][1] = ROcp4_45;
    sens->R[2][2] = ROcp4_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp4_75;
    sens->R[3][2] = ROcp4_85;
    sens->R[3][3] = C5;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = OMcp4_15;
    sens->OM[2] = OMcp4_25;
    sens->OM[3] = qd[4];
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
    sens->OMP[1] = OPcp4_15;
    sens->OMP[2] = OPcp4_25;
    sens->OMP[3] = qdd[4];
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


    ROcp5_45 = -S4*C5;
    ROcp5_55 = C4*C5;
    ROcp5_75 = S4*S5;
    ROcp5_85 = -C4*S5;
    ROcp5_16 = -(ROcp5_75*S6-C4*C6);
    ROcp5_26 = -(ROcp5_85*S6-S4*C6);
    ROcp5_36 = -C5*S6;
    ROcp5_76 = ROcp5_75*C6+C4*S6;
    ROcp5_86 = ROcp5_85*C6+S4*S6;
    ROcp5_96 = C5*C6;
    OMcp5_15 = qd[5]*C4;
    OMcp5_25 = qd[5]*S4;
    OMcp5_16 = OMcp5_15+ROcp5_45*qd[6];
    OMcp5_26 = OMcp5_25+ROcp5_55*qd[6];
    OMcp5_36 = qd[4]+qd[6]*S5;
    OPcp5_16 = ROcp5_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_25*S5-ROcp5_55*qd[4]);
    OPcp5_26 = ROcp5_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_15*S5-ROcp5_45*qd[4]);
    OPcp5_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = q[1];
    sens->P[2] = q[2];
    sens->P[3] = q[3];
    sens->R[1][1] = ROcp5_16;
    sens->R[1][2] = ROcp5_26;
    sens->R[1][3] = ROcp5_36;
    sens->R[2][1] = ROcp5_45;
    sens->R[2][2] = ROcp5_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp5_76;
    sens->R[3][2] = ROcp5_86;
    sens->R[3][3] = ROcp5_96;
    sens->V[1] = qd[1];
    sens->V[2] = qd[2];
    sens->V[3] = qd[3];
    sens->OM[1] = OMcp5_16;
    sens->OM[2] = OMcp5_26;
    sens->OM[3] = OMcp5_36;
    sens->A[1] = qdd[1];
    sens->A[2] = qdd[2];
    sens->A[3] = qdd[3];
    sens->OMP[1] = OPcp5_16;
    sens->OMP[2] = OPcp5_26;
    sens->OMP[3] = OPcp5_36;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


    ROcp6_45 = -S4*C5;
    ROcp6_55 = C4*C5;
    ROcp6_75 = S4*S5;
    ROcp6_85 = -C4*S5;
    ROcp6_16 = -(ROcp6_75*S6-C4*C6);
    ROcp6_26 = -(ROcp6_85*S6-S4*C6);
    ROcp6_36 = -C5*S6;
    ROcp6_76 = ROcp6_75*C6+C4*S6;
    ROcp6_86 = ROcp6_85*C6+S4*S6;
    ROcp6_96 = C5*C6;
    OMcp6_15 = qd[5]*C4;
    OMcp6_25 = qd[5]*S4;
    OMcp6_16 = OMcp6_15+ROcp6_45*qd[6];
    OMcp6_26 = OMcp6_25+ROcp6_55*qd[6];
    OMcp6_36 = qd[4]+qd[6]*S5;
    OPcp6_16 = ROcp6_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_25*S5-ROcp6_55*qd[4]);
    OPcp6_26 = ROcp6_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_15*S5-ROcp6_45*qd[4]);
    OPcp6_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_7_0_2 = = 
 
// Sensor Kinematics 


    ROcp6_47 = ROcp6_45*C7+ROcp6_76*S7;
    ROcp6_57 = ROcp6_55*C7+ROcp6_86*S7;
    ROcp6_67 = ROcp6_96*S7+S5*C7;
    ROcp6_77 = -(ROcp6_45*S7-ROcp6_76*C7);
    ROcp6_87 = -(ROcp6_55*S7-ROcp6_86*C7);
    ROcp6_97 = ROcp6_96*C7-S5*S7;
    RLcp6_17 = ROcp6_16*s->dpt[1][1]+ROcp6_45*s->dpt[2][1]+ROcp6_76*s->dpt[3][1];
    RLcp6_27 = ROcp6_26*s->dpt[1][1]+ROcp6_55*s->dpt[2][1]+ROcp6_86*s->dpt[3][1];
    RLcp6_37 = ROcp6_36*s->dpt[1][1]+ROcp6_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    POcp6_17 = RLcp6_17+q[1];
    POcp6_27 = RLcp6_27+q[2];
    POcp6_37 = RLcp6_37+q[3];
    OMcp6_17 = OMcp6_16+ROcp6_16*qd[7];
    OMcp6_27 = OMcp6_26+ROcp6_26*qd[7];
    OMcp6_37 = OMcp6_36+ROcp6_36*qd[7];
    ORcp6_17 = OMcp6_26*RLcp6_37-OMcp6_36*RLcp6_27;
    ORcp6_27 = -(OMcp6_16*RLcp6_37-OMcp6_36*RLcp6_17);
    ORcp6_37 = OMcp6_16*RLcp6_27-OMcp6_26*RLcp6_17;
    VIcp6_17 = ORcp6_17+qd[1];
    VIcp6_27 = ORcp6_27+qd[2];
    VIcp6_37 = ORcp6_37+qd[3];
    OPcp6_17 = OPcp6_16+ROcp6_16*qdd[7]+qd[7]*(OMcp6_26*ROcp6_36-OMcp6_36*ROcp6_26);
    OPcp6_27 = OPcp6_26+ROcp6_26*qdd[7]-qd[7]*(OMcp6_16*ROcp6_36-OMcp6_36*ROcp6_16);
    OPcp6_37 = OPcp6_36+ROcp6_36*qdd[7]+qd[7]*(OMcp6_16*ROcp6_26-OMcp6_26*ROcp6_16);
    ACcp6_17 = qdd[1]+OMcp6_26*ORcp6_37-OMcp6_36*ORcp6_27+OPcp6_26*RLcp6_37-OPcp6_36*RLcp6_27;
    ACcp6_27 = qdd[2]-OMcp6_16*ORcp6_37+OMcp6_36*ORcp6_17-OPcp6_16*RLcp6_37+OPcp6_36*RLcp6_17;
    ACcp6_37 = qdd[3]+OMcp6_16*ORcp6_27-OMcp6_26*ORcp6_17+OPcp6_16*RLcp6_27-OPcp6_26*RLcp6_17;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_17;
    sens->P[2] = POcp6_27;
    sens->P[3] = POcp6_37;
    sens->R[1][1] = ROcp6_16;
    sens->R[1][2] = ROcp6_26;
    sens->R[1][3] = ROcp6_36;
    sens->R[2][1] = ROcp6_47;
    sens->R[2][2] = ROcp6_57;
    sens->R[2][3] = ROcp6_67;
    sens->R[3][1] = ROcp6_77;
    sens->R[3][2] = ROcp6_87;
    sens->R[3][3] = ROcp6_97;
    sens->V[1] = VIcp6_17;
    sens->V[2] = VIcp6_27;
    sens->V[3] = VIcp6_37;
    sens->OM[1] = OMcp6_17;
    sens->OM[2] = OMcp6_27;
    sens->OM[3] = OMcp6_37;
    sens->A[1] = ACcp6_17;
    sens->A[2] = ACcp6_27;
    sens->A[3] = ACcp6_37;
    sens->OMP[1] = OPcp6_17;
    sens->OMP[2] = OPcp6_27;
    sens->OMP[3] = OPcp6_37;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


    ROcp7_45 = -S4*C5;
    ROcp7_55 = C4*C5;
    ROcp7_75 = S4*S5;
    ROcp7_85 = -C4*S5;
    ROcp7_16 = -(ROcp7_75*S6-C4*C6);
    ROcp7_26 = -(ROcp7_85*S6-S4*C6);
    ROcp7_36 = -C5*S6;
    ROcp7_76 = ROcp7_75*C6+C4*S6;
    ROcp7_86 = ROcp7_85*C6+S4*S6;
    ROcp7_96 = C5*C6;
    OMcp7_15 = qd[5]*C4;
    OMcp7_25 = qd[5]*S4;
    OMcp7_16 = OMcp7_15+ROcp7_45*qd[6];
    OMcp7_26 = OMcp7_25+ROcp7_55*qd[6];
    OMcp7_36 = qd[4]+qd[6]*S5;
    OPcp7_16 = ROcp7_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_25*S5-ROcp7_55*qd[4]);
    OPcp7_26 = ROcp7_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_15*S5-ROcp7_45*qd[4]);
    OPcp7_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_8_0_2 = = 
 
// Sensor Kinematics 


    ROcp7_47 = ROcp7_45*C7+ROcp7_76*S7;
    ROcp7_57 = ROcp7_55*C7+ROcp7_86*S7;
    ROcp7_67 = ROcp7_96*S7+S5*C7;
    ROcp7_77 = -(ROcp7_45*S7-ROcp7_76*C7);
    ROcp7_87 = -(ROcp7_55*S7-ROcp7_86*C7);
    ROcp7_97 = ROcp7_96*C7-S5*S7;
    ROcp7_18 = ROcp7_16*C8-ROcp7_77*S8;
    ROcp7_28 = ROcp7_26*C8-ROcp7_87*S8;
    ROcp7_38 = ROcp7_36*C8-ROcp7_97*S8;
    ROcp7_78 = ROcp7_16*S8+ROcp7_77*C8;
    ROcp7_88 = ROcp7_26*S8+ROcp7_87*C8;
    ROcp7_98 = ROcp7_36*S8+ROcp7_97*C8;
    RLcp7_17 = ROcp7_16*s->dpt[1][1]+ROcp7_45*s->dpt[2][1]+ROcp7_76*s->dpt[3][1];
    RLcp7_27 = ROcp7_26*s->dpt[1][1]+ROcp7_55*s->dpt[2][1]+ROcp7_86*s->dpt[3][1];
    RLcp7_37 = ROcp7_36*s->dpt[1][1]+ROcp7_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp7_17 = OMcp7_16+ROcp7_16*qd[7];
    OMcp7_27 = OMcp7_26+ROcp7_26*qd[7];
    OMcp7_37 = OMcp7_36+ROcp7_36*qd[7];
    ORcp7_17 = OMcp7_26*RLcp7_37-OMcp7_36*RLcp7_27;
    ORcp7_27 = -(OMcp7_16*RLcp7_37-OMcp7_36*RLcp7_17);
    ORcp7_37 = OMcp7_16*RLcp7_27-OMcp7_26*RLcp7_17;
    OPcp7_17 = OPcp7_16+ROcp7_16*qdd[7]+qd[7]*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26);
    OPcp7_27 = OPcp7_26+ROcp7_26*qdd[7]-qd[7]*(OMcp7_16*ROcp7_36-OMcp7_36*ROcp7_16);
    OPcp7_37 = OPcp7_36+ROcp7_36*qdd[7]+qd[7]*(OMcp7_16*ROcp7_26-OMcp7_26*ROcp7_16);
    RLcp7_18 = ROcp7_47*s->dpt[2][16];
    RLcp7_28 = ROcp7_57*s->dpt[2][16];
    RLcp7_38 = ROcp7_67*s->dpt[2][16];
    POcp7_18 = RLcp7_17+RLcp7_18+q[1];
    POcp7_28 = RLcp7_27+RLcp7_28+q[2];
    POcp7_38 = RLcp7_37+RLcp7_38+q[3];
    OMcp7_18 = OMcp7_17+ROcp7_47*qd[8];
    OMcp7_28 = OMcp7_27+ROcp7_57*qd[8];
    OMcp7_38 = OMcp7_37+ROcp7_67*qd[8];
    ORcp7_18 = OMcp7_27*RLcp7_38-OMcp7_37*RLcp7_28;
    ORcp7_28 = -(OMcp7_17*RLcp7_38-OMcp7_37*RLcp7_18);
    ORcp7_38 = OMcp7_17*RLcp7_28-OMcp7_27*RLcp7_18;
    VIcp7_18 = ORcp7_17+ORcp7_18+qd[1];
    VIcp7_28 = ORcp7_27+ORcp7_28+qd[2];
    VIcp7_38 = ORcp7_37+ORcp7_38+qd[3];
    OPcp7_18 = OPcp7_17+ROcp7_47*qdd[8]+qd[8]*(OMcp7_27*ROcp7_67-OMcp7_37*ROcp7_57);
    OPcp7_28 = OPcp7_27+ROcp7_57*qdd[8]-qd[8]*(OMcp7_17*ROcp7_67-OMcp7_37*ROcp7_47);
    OPcp7_38 = OPcp7_37+ROcp7_67*qdd[8]+qd[8]*(OMcp7_17*ROcp7_57-OMcp7_27*ROcp7_47);
    ACcp7_18 = qdd[1]+OMcp7_26*ORcp7_37+OMcp7_27*ORcp7_38-OMcp7_36*ORcp7_27-OMcp7_37*ORcp7_28+OPcp7_26*RLcp7_37+OPcp7_27*
 RLcp7_38-OPcp7_36*RLcp7_27-OPcp7_37*RLcp7_28;
    ACcp7_28 = qdd[2]-OMcp7_16*ORcp7_37-OMcp7_17*ORcp7_38+OMcp7_36*ORcp7_17+OMcp7_37*ORcp7_18-OPcp7_16*RLcp7_37-OPcp7_17*
 RLcp7_38+OPcp7_36*RLcp7_17+OPcp7_37*RLcp7_18;
    ACcp7_38 = qdd[3]+OMcp7_16*ORcp7_27+OMcp7_17*ORcp7_28-OMcp7_26*ORcp7_17-OMcp7_27*ORcp7_18+OPcp7_16*RLcp7_27+OPcp7_17*
 RLcp7_28-OPcp7_26*RLcp7_17-OPcp7_27*RLcp7_18;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_18;
    sens->P[2] = POcp7_28;
    sens->P[3] = POcp7_38;
    sens->R[1][1] = ROcp7_18;
    sens->R[1][2] = ROcp7_28;
    sens->R[1][3] = ROcp7_38;
    sens->R[2][1] = ROcp7_47;
    sens->R[2][2] = ROcp7_57;
    sens->R[2][3] = ROcp7_67;
    sens->R[3][1] = ROcp7_78;
    sens->R[3][2] = ROcp7_88;
    sens->R[3][3] = ROcp7_98;
    sens->V[1] = VIcp7_18;
    sens->V[2] = VIcp7_28;
    sens->V[3] = VIcp7_38;
    sens->OM[1] = OMcp7_18;
    sens->OM[2] = OMcp7_28;
    sens->OM[3] = OMcp7_38;
    sens->A[1] = ACcp7_18;
    sens->A[2] = ACcp7_28;
    sens->A[3] = ACcp7_38;
    sens->OMP[1] = OPcp7_18;
    sens->OMP[2] = OPcp7_28;
    sens->OMP[3] = OPcp7_38;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


    ROcp8_45 = -S4*C5;
    ROcp8_55 = C4*C5;
    ROcp8_75 = S4*S5;
    ROcp8_85 = -C4*S5;
    ROcp8_16 = -(ROcp8_75*S6-C4*C6);
    ROcp8_26 = -(ROcp8_85*S6-S4*C6);
    ROcp8_36 = -C5*S6;
    ROcp8_76 = ROcp8_75*C6+C4*S6;
    ROcp8_86 = ROcp8_85*C6+S4*S6;
    ROcp8_96 = C5*C6;
    OMcp8_15 = qd[5]*C4;
    OMcp8_25 = qd[5]*S4;
    OMcp8_16 = OMcp8_15+ROcp8_45*qd[6];
    OMcp8_26 = OMcp8_25+ROcp8_55*qd[6];
    OMcp8_36 = qd[4]+qd[6]*S5;
    OPcp8_16 = ROcp8_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp8_25*S5-ROcp8_55*qd[4]);
    OPcp8_26 = ROcp8_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp8_15*S5-ROcp8_45*qd[4]);
    OPcp8_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_9_0_2 = = 
 
// Sensor Kinematics 


    ROcp8_47 = ROcp8_45*C7+ROcp8_76*S7;
    ROcp8_57 = ROcp8_55*C7+ROcp8_86*S7;
    ROcp8_67 = ROcp8_96*S7+S5*C7;
    ROcp8_77 = -(ROcp8_45*S7-ROcp8_76*C7);
    ROcp8_87 = -(ROcp8_55*S7-ROcp8_86*C7);
    ROcp8_97 = ROcp8_96*C7-S5*S7;
    ROcp8_18 = ROcp8_16*C8-ROcp8_77*S8;
    ROcp8_28 = ROcp8_26*C8-ROcp8_87*S8;
    ROcp8_38 = ROcp8_36*C8-ROcp8_97*S8;
    ROcp8_78 = ROcp8_16*S8+ROcp8_77*C8;
    ROcp8_88 = ROcp8_26*S8+ROcp8_87*C8;
    ROcp8_98 = ROcp8_36*S8+ROcp8_97*C8;
    ROcp8_49 = ROcp8_47*C9+ROcp8_78*S9;
    ROcp8_59 = ROcp8_57*C9+ROcp8_88*S9;
    ROcp8_69 = ROcp8_67*C9+ROcp8_98*S9;
    ROcp8_79 = -(ROcp8_47*S9-ROcp8_78*C9);
    ROcp8_89 = -(ROcp8_57*S9-ROcp8_88*C9);
    ROcp8_99 = -(ROcp8_67*S9-ROcp8_98*C9);
    RLcp8_17 = ROcp8_16*s->dpt[1][1]+ROcp8_45*s->dpt[2][1]+ROcp8_76*s->dpt[3][1];
    RLcp8_27 = ROcp8_26*s->dpt[1][1]+ROcp8_55*s->dpt[2][1]+ROcp8_86*s->dpt[3][1];
    RLcp8_37 = ROcp8_36*s->dpt[1][1]+ROcp8_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp8_17 = OMcp8_16+ROcp8_16*qd[7];
    OMcp8_27 = OMcp8_26+ROcp8_26*qd[7];
    OMcp8_37 = OMcp8_36+ROcp8_36*qd[7];
    ORcp8_17 = OMcp8_26*RLcp8_37-OMcp8_36*RLcp8_27;
    ORcp8_27 = -(OMcp8_16*RLcp8_37-OMcp8_36*RLcp8_17);
    ORcp8_37 = OMcp8_16*RLcp8_27-OMcp8_26*RLcp8_17;
    OPcp8_17 = OPcp8_16+ROcp8_16*qdd[7]+qd[7]*(OMcp8_26*ROcp8_36-OMcp8_36*ROcp8_26);
    OPcp8_27 = OPcp8_26+ROcp8_26*qdd[7]-qd[7]*(OMcp8_16*ROcp8_36-OMcp8_36*ROcp8_16);
    OPcp8_37 = OPcp8_36+ROcp8_36*qdd[7]+qd[7]*(OMcp8_16*ROcp8_26-OMcp8_26*ROcp8_16);
    RLcp8_18 = ROcp8_47*s->dpt[2][16];
    RLcp8_28 = ROcp8_57*s->dpt[2][16];
    RLcp8_38 = ROcp8_67*s->dpt[2][16];
    POcp8_18 = RLcp8_17+RLcp8_18+q[1];
    POcp8_28 = RLcp8_27+RLcp8_28+q[2];
    POcp8_38 = RLcp8_37+RLcp8_38+q[3];
    OMcp8_18 = OMcp8_17+ROcp8_47*qd[8];
    OMcp8_28 = OMcp8_27+ROcp8_57*qd[8];
    OMcp8_38 = OMcp8_37+ROcp8_67*qd[8];
    ORcp8_18 = OMcp8_27*RLcp8_38-OMcp8_37*RLcp8_28;
    ORcp8_28 = -(OMcp8_17*RLcp8_38-OMcp8_37*RLcp8_18);
    ORcp8_38 = OMcp8_17*RLcp8_28-OMcp8_27*RLcp8_18;
    VIcp8_18 = ORcp8_17+ORcp8_18+qd[1];
    VIcp8_28 = ORcp8_27+ORcp8_28+qd[2];
    VIcp8_38 = ORcp8_37+ORcp8_38+qd[3];
    ACcp8_18 = qdd[1]+OMcp8_26*ORcp8_37+OMcp8_27*ORcp8_38-OMcp8_36*ORcp8_27-OMcp8_37*ORcp8_28+OPcp8_26*RLcp8_37+OPcp8_27*
 RLcp8_38-OPcp8_36*RLcp8_27-OPcp8_37*RLcp8_28;
    ACcp8_28 = qdd[2]-OMcp8_16*ORcp8_37-OMcp8_17*ORcp8_38+OMcp8_36*ORcp8_17+OMcp8_37*ORcp8_18-OPcp8_16*RLcp8_37-OPcp8_17*
 RLcp8_38+OPcp8_36*RLcp8_17+OPcp8_37*RLcp8_18;
    ACcp8_38 = qdd[3]+OMcp8_16*ORcp8_27+OMcp8_17*ORcp8_28-OMcp8_26*ORcp8_17-OMcp8_27*ORcp8_18+OPcp8_16*RLcp8_27+OPcp8_17*
 RLcp8_28-OPcp8_26*RLcp8_17-OPcp8_27*RLcp8_18;
    OMcp8_19 = OMcp8_18+ROcp8_18*qd[9];
    OMcp8_29 = OMcp8_28+ROcp8_28*qd[9];
    OMcp8_39 = OMcp8_38+ROcp8_38*qd[9];
    OPcp8_19 = OPcp8_17+ROcp8_18*qdd[9]+ROcp8_47*qdd[8]+qd[8]*(OMcp8_27*ROcp8_67-OMcp8_37*ROcp8_57)+qd[9]*(OMcp8_28*
 ROcp8_38-OMcp8_38*ROcp8_28);
    OPcp8_29 = OPcp8_27+ROcp8_28*qdd[9]+ROcp8_57*qdd[8]-qd[8]*(OMcp8_17*ROcp8_67-OMcp8_37*ROcp8_47)-qd[9]*(OMcp8_18*
 ROcp8_38-OMcp8_38*ROcp8_18);
    OPcp8_39 = OPcp8_37+ROcp8_38*qdd[9]+ROcp8_67*qdd[8]+qd[8]*(OMcp8_17*ROcp8_57-OMcp8_27*ROcp8_47)+qd[9]*(OMcp8_18*
 ROcp8_28-OMcp8_28*ROcp8_18);

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_18;
    sens->P[2] = POcp8_28;
    sens->P[3] = POcp8_38;
    sens->R[1][1] = ROcp8_18;
    sens->R[1][2] = ROcp8_28;
    sens->R[1][3] = ROcp8_38;
    sens->R[2][1] = ROcp8_49;
    sens->R[2][2] = ROcp8_59;
    sens->R[2][3] = ROcp8_69;
    sens->R[3][1] = ROcp8_79;
    sens->R[3][2] = ROcp8_89;
    sens->R[3][3] = ROcp8_99;
    sens->V[1] = VIcp8_18;
    sens->V[2] = VIcp8_28;
    sens->V[3] = VIcp8_38;
    sens->OM[1] = OMcp8_19;
    sens->OM[2] = OMcp8_29;
    sens->OM[3] = OMcp8_39;
    sens->A[1] = ACcp8_18;
    sens->A[2] = ACcp8_28;
    sens->A[3] = ACcp8_38;
    sens->OMP[1] = OPcp8_19;
    sens->OMP[2] = OPcp8_29;
    sens->OMP[3] = OPcp8_39;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


    ROcp9_45 = -S4*C5;
    ROcp9_55 = C4*C5;
    ROcp9_75 = S4*S5;
    ROcp9_85 = -C4*S5;
    ROcp9_16 = -(ROcp9_75*S6-C4*C6);
    ROcp9_26 = -(ROcp9_85*S6-S4*C6);
    ROcp9_36 = -C5*S6;
    ROcp9_76 = ROcp9_75*C6+C4*S6;
    ROcp9_86 = ROcp9_85*C6+S4*S6;
    ROcp9_96 = C5*C6;
    OMcp9_15 = qd[5]*C4;
    OMcp9_25 = qd[5]*S4;
    OMcp9_16 = OMcp9_15+ROcp9_45*qd[6];
    OMcp9_26 = OMcp9_25+ROcp9_55*qd[6];
    OMcp9_36 = qd[4]+qd[6]*S5;
    OPcp9_16 = ROcp9_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp9_25*S5-ROcp9_55*qd[4]);
    OPcp9_26 = ROcp9_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp9_15*S5-ROcp9_45*qd[4]);
    OPcp9_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_10_0_2 = = 
 
// Sensor Kinematics 


    ROcp9_47 = ROcp9_45*C7+ROcp9_76*S7;
    ROcp9_57 = ROcp9_55*C7+ROcp9_86*S7;
    ROcp9_67 = ROcp9_96*S7+S5*C7;
    ROcp9_77 = -(ROcp9_45*S7-ROcp9_76*C7);
    ROcp9_87 = -(ROcp9_55*S7-ROcp9_86*C7);
    ROcp9_97 = ROcp9_96*C7-S5*S7;
    ROcp9_18 = ROcp9_16*C8-ROcp9_77*S8;
    ROcp9_28 = ROcp9_26*C8-ROcp9_87*S8;
    ROcp9_38 = ROcp9_36*C8-ROcp9_97*S8;
    ROcp9_78 = ROcp9_16*S8+ROcp9_77*C8;
    ROcp9_88 = ROcp9_26*S8+ROcp9_87*C8;
    ROcp9_98 = ROcp9_36*S8+ROcp9_97*C8;
    ROcp9_49 = ROcp9_47*C9+ROcp9_78*S9;
    ROcp9_59 = ROcp9_57*C9+ROcp9_88*S9;
    ROcp9_69 = ROcp9_67*C9+ROcp9_98*S9;
    ROcp9_79 = -(ROcp9_47*S9-ROcp9_78*C9);
    ROcp9_89 = -(ROcp9_57*S9-ROcp9_88*C9);
    ROcp9_99 = -(ROcp9_67*S9-ROcp9_98*C9);
    ROcp9_110 = ROcp9_18*C10+ROcp9_49*S10;
    ROcp9_210 = ROcp9_28*C10+ROcp9_59*S10;
    ROcp9_310 = ROcp9_38*C10+ROcp9_69*S10;
    ROcp9_410 = -(ROcp9_18*S10-ROcp9_49*C10);
    ROcp9_510 = -(ROcp9_28*S10-ROcp9_59*C10);
    ROcp9_610 = -(ROcp9_38*S10-ROcp9_69*C10);
    RLcp9_17 = ROcp9_16*s->dpt[1][1]+ROcp9_45*s->dpt[2][1]+ROcp9_76*s->dpt[3][1];
    RLcp9_27 = ROcp9_26*s->dpt[1][1]+ROcp9_55*s->dpt[2][1]+ROcp9_86*s->dpt[3][1];
    RLcp9_37 = ROcp9_36*s->dpt[1][1]+ROcp9_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp9_17 = OMcp9_16+ROcp9_16*qd[7];
    OMcp9_27 = OMcp9_26+ROcp9_26*qd[7];
    OMcp9_37 = OMcp9_36+ROcp9_36*qd[7];
    ORcp9_17 = OMcp9_26*RLcp9_37-OMcp9_36*RLcp9_27;
    ORcp9_27 = -(OMcp9_16*RLcp9_37-OMcp9_36*RLcp9_17);
    ORcp9_37 = OMcp9_16*RLcp9_27-OMcp9_26*RLcp9_17;
    OPcp9_17 = OPcp9_16+ROcp9_16*qdd[7]+qd[7]*(OMcp9_26*ROcp9_36-OMcp9_36*ROcp9_26);
    OPcp9_27 = OPcp9_26+ROcp9_26*qdd[7]-qd[7]*(OMcp9_16*ROcp9_36-OMcp9_36*ROcp9_16);
    OPcp9_37 = OPcp9_36+ROcp9_36*qdd[7]+qd[7]*(OMcp9_16*ROcp9_26-OMcp9_26*ROcp9_16);
    RLcp9_18 = ROcp9_47*s->dpt[2][16];
    RLcp9_28 = ROcp9_57*s->dpt[2][16];
    RLcp9_38 = ROcp9_67*s->dpt[2][16];
    POcp9_18 = RLcp9_17+RLcp9_18+q[1];
    POcp9_28 = RLcp9_27+RLcp9_28+q[2];
    POcp9_38 = RLcp9_37+RLcp9_38+q[3];
    OMcp9_18 = OMcp9_17+ROcp9_47*qd[8];
    OMcp9_28 = OMcp9_27+ROcp9_57*qd[8];
    OMcp9_38 = OMcp9_37+ROcp9_67*qd[8];
    ORcp9_18 = OMcp9_27*RLcp9_38-OMcp9_37*RLcp9_28;
    ORcp9_28 = -(OMcp9_17*RLcp9_38-OMcp9_37*RLcp9_18);
    ORcp9_38 = OMcp9_17*RLcp9_28-OMcp9_27*RLcp9_18;
    VIcp9_18 = ORcp9_17+ORcp9_18+qd[1];
    VIcp9_28 = ORcp9_27+ORcp9_28+qd[2];
    VIcp9_38 = ORcp9_37+ORcp9_38+qd[3];
    ACcp9_18 = qdd[1]+OMcp9_26*ORcp9_37+OMcp9_27*ORcp9_38-OMcp9_36*ORcp9_27-OMcp9_37*ORcp9_28+OPcp9_26*RLcp9_37+OPcp9_27*
 RLcp9_38-OPcp9_36*RLcp9_27-OPcp9_37*RLcp9_28;
    ACcp9_28 = qdd[2]-OMcp9_16*ORcp9_37-OMcp9_17*ORcp9_38+OMcp9_36*ORcp9_17+OMcp9_37*ORcp9_18-OPcp9_16*RLcp9_37-OPcp9_17*
 RLcp9_38+OPcp9_36*RLcp9_17+OPcp9_37*RLcp9_18;
    ACcp9_38 = qdd[3]+OMcp9_16*ORcp9_27+OMcp9_17*ORcp9_28-OMcp9_26*ORcp9_17-OMcp9_27*ORcp9_18+OPcp9_16*RLcp9_27+OPcp9_17*
 RLcp9_28-OPcp9_26*RLcp9_17-OPcp9_27*RLcp9_18;
    OMcp9_19 = OMcp9_18+ROcp9_18*qd[9];
    OMcp9_29 = OMcp9_28+ROcp9_28*qd[9];
    OMcp9_39 = OMcp9_38+ROcp9_38*qd[9];
    OMcp9_110 = OMcp9_19+ROcp9_79*qd[10];
    OMcp9_210 = OMcp9_29+ROcp9_89*qd[10];
    OMcp9_310 = OMcp9_39+ROcp9_99*qd[10];
    OPcp9_110 = OPcp9_17+ROcp9_18*qdd[9]+ROcp9_47*qdd[8]+ROcp9_79*qdd[10]+qd[10]*(OMcp9_29*ROcp9_99-OMcp9_39*ROcp9_89)+
 qd[8]*(OMcp9_27*ROcp9_67-OMcp9_37*ROcp9_57)+qd[9]*(OMcp9_28*ROcp9_38-OMcp9_38*ROcp9_28);
    OPcp9_210 = OPcp9_27+ROcp9_28*qdd[9]+ROcp9_57*qdd[8]+ROcp9_89*qdd[10]-qd[10]*(OMcp9_19*ROcp9_99-OMcp9_39*ROcp9_79)-
 qd[8]*(OMcp9_17*ROcp9_67-OMcp9_37*ROcp9_47)-qd[9]*(OMcp9_18*ROcp9_38-OMcp9_38*ROcp9_18);
    OPcp9_310 = OPcp9_37+ROcp9_38*qdd[9]+ROcp9_67*qdd[8]+ROcp9_99*qdd[10]+qd[10]*(OMcp9_19*ROcp9_89-OMcp9_29*ROcp9_79)+
 qd[8]*(OMcp9_17*ROcp9_57-OMcp9_27*ROcp9_47)+qd[9]*(OMcp9_18*ROcp9_28-OMcp9_28*ROcp9_18);

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_18;
    sens->P[2] = POcp9_28;
    sens->P[3] = POcp9_38;
    sens->R[1][1] = ROcp9_110;
    sens->R[1][2] = ROcp9_210;
    sens->R[1][3] = ROcp9_310;
    sens->R[2][1] = ROcp9_410;
    sens->R[2][2] = ROcp9_510;
    sens->R[2][3] = ROcp9_610;
    sens->R[3][1] = ROcp9_79;
    sens->R[3][2] = ROcp9_89;
    sens->R[3][3] = ROcp9_99;
    sens->V[1] = VIcp9_18;
    sens->V[2] = VIcp9_28;
    sens->V[3] = VIcp9_38;
    sens->OM[1] = OMcp9_110;
    sens->OM[2] = OMcp9_210;
    sens->OM[3] = OMcp9_310;
    sens->A[1] = ACcp9_18;
    sens->A[2] = ACcp9_28;
    sens->A[3] = ACcp9_38;
    sens->OMP[1] = OPcp9_110;
    sens->OMP[2] = OPcp9_210;
    sens->OMP[3] = OPcp9_310;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
// Sensor Kinematics 


    ROcp10_45 = -S4*C5;
    ROcp10_55 = C4*C5;
    ROcp10_75 = S4*S5;
    ROcp10_85 = -C4*S5;
    ROcp10_16 = -(ROcp10_75*S6-C4*C6);
    ROcp10_26 = -(ROcp10_85*S6-S4*C6);
    ROcp10_36 = -C5*S6;
    ROcp10_76 = ROcp10_75*C6+C4*S6;
    ROcp10_86 = ROcp10_85*C6+S4*S6;
    ROcp10_96 = C5*C6;
    OMcp10_15 = qd[5]*C4;
    OMcp10_25 = qd[5]*S4;
    OMcp10_16 = OMcp10_15+ROcp10_45*qd[6];
    OMcp10_26 = OMcp10_25+ROcp10_55*qd[6];
    OMcp10_36 = qd[4]+qd[6]*S5;
    OPcp10_16 = ROcp10_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp10_25*S5-ROcp10_55*qd[4]);
    OPcp10_26 = ROcp10_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp10_15*S5-ROcp10_45*qd[4]);
    OPcp10_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_11_0_2 = = 
 
// Sensor Kinematics 


    ROcp10_47 = ROcp10_45*C7+ROcp10_76*S7;
    ROcp10_57 = ROcp10_55*C7+ROcp10_86*S7;
    ROcp10_67 = ROcp10_96*S7+S5*C7;
    ROcp10_77 = -(ROcp10_45*S7-ROcp10_76*C7);
    ROcp10_87 = -(ROcp10_55*S7-ROcp10_86*C7);
    ROcp10_97 = ROcp10_96*C7-S5*S7;
    ROcp10_18 = ROcp10_16*C8-ROcp10_77*S8;
    ROcp10_28 = ROcp10_26*C8-ROcp10_87*S8;
    ROcp10_38 = ROcp10_36*C8-ROcp10_97*S8;
    ROcp10_78 = ROcp10_16*S8+ROcp10_77*C8;
    ROcp10_88 = ROcp10_26*S8+ROcp10_87*C8;
    ROcp10_98 = ROcp10_36*S8+ROcp10_97*C8;
    ROcp10_49 = ROcp10_47*C9+ROcp10_78*S9;
    ROcp10_59 = ROcp10_57*C9+ROcp10_88*S9;
    ROcp10_69 = ROcp10_67*C9+ROcp10_98*S9;
    ROcp10_79 = -(ROcp10_47*S9-ROcp10_78*C9);
    ROcp10_89 = -(ROcp10_57*S9-ROcp10_88*C9);
    ROcp10_99 = -(ROcp10_67*S9-ROcp10_98*C9);
    ROcp10_110 = ROcp10_18*C10+ROcp10_49*S10;
    ROcp10_210 = ROcp10_28*C10+ROcp10_59*S10;
    ROcp10_310 = ROcp10_38*C10+ROcp10_69*S10;
    ROcp10_410 = -(ROcp10_18*S10-ROcp10_49*C10);
    ROcp10_510 = -(ROcp10_28*S10-ROcp10_59*C10);
    ROcp10_610 = -(ROcp10_38*S10-ROcp10_69*C10);
    ROcp10_111 = ROcp10_110*C11-ROcp10_79*S11;
    ROcp10_211 = ROcp10_210*C11-ROcp10_89*S11;
    ROcp10_311 = ROcp10_310*C11-ROcp10_99*S11;
    ROcp10_711 = ROcp10_110*S11+ROcp10_79*C11;
    ROcp10_811 = ROcp10_210*S11+ROcp10_89*C11;
    ROcp10_911 = ROcp10_310*S11+ROcp10_99*C11;
    RLcp10_17 = ROcp10_16*s->dpt[1][1]+ROcp10_45*s->dpt[2][1]+ROcp10_76*s->dpt[3][1];
    RLcp10_27 = ROcp10_26*s->dpt[1][1]+ROcp10_55*s->dpt[2][1]+ROcp10_86*s->dpt[3][1];
    RLcp10_37 = ROcp10_36*s->dpt[1][1]+ROcp10_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp10_17 = OMcp10_16+ROcp10_16*qd[7];
    OMcp10_27 = OMcp10_26+ROcp10_26*qd[7];
    OMcp10_37 = OMcp10_36+ROcp10_36*qd[7];
    ORcp10_17 = OMcp10_26*RLcp10_37-OMcp10_36*RLcp10_27;
    ORcp10_27 = -(OMcp10_16*RLcp10_37-OMcp10_36*RLcp10_17);
    ORcp10_37 = OMcp10_16*RLcp10_27-OMcp10_26*RLcp10_17;
    OPcp10_17 = OPcp10_16+ROcp10_16*qdd[7]+qd[7]*(OMcp10_26*ROcp10_36-OMcp10_36*ROcp10_26);
    OPcp10_27 = OPcp10_26+ROcp10_26*qdd[7]-qd[7]*(OMcp10_16*ROcp10_36-OMcp10_36*ROcp10_16);
    OPcp10_37 = OPcp10_36+ROcp10_36*qdd[7]+qd[7]*(OMcp10_16*ROcp10_26-OMcp10_26*ROcp10_16);
    RLcp10_18 = ROcp10_47*s->dpt[2][16];
    RLcp10_28 = ROcp10_57*s->dpt[2][16];
    RLcp10_38 = ROcp10_67*s->dpt[2][16];
    OMcp10_18 = OMcp10_17+ROcp10_47*qd[8];
    OMcp10_28 = OMcp10_27+ROcp10_57*qd[8];
    OMcp10_38 = OMcp10_37+ROcp10_67*qd[8];
    ORcp10_18 = OMcp10_27*RLcp10_38-OMcp10_37*RLcp10_28;
    ORcp10_28 = -(OMcp10_17*RLcp10_38-OMcp10_37*RLcp10_18);
    ORcp10_38 = OMcp10_17*RLcp10_28-OMcp10_27*RLcp10_18;
    OMcp10_19 = OMcp10_18+ROcp10_18*qd[9];
    OMcp10_29 = OMcp10_28+ROcp10_28*qd[9];
    OMcp10_39 = OMcp10_38+ROcp10_38*qd[9];
    OMcp10_110 = OMcp10_19+ROcp10_79*qd[10];
    OMcp10_210 = OMcp10_29+ROcp10_89*qd[10];
    OMcp10_310 = OMcp10_39+ROcp10_99*qd[10];
    OPcp10_110 = OPcp10_17+ROcp10_18*qdd[9]+ROcp10_47*qdd[8]+ROcp10_79*qdd[10]+qd[10]*(OMcp10_29*ROcp10_99-OMcp10_39*
 ROcp10_89)+qd[8]*(OMcp10_27*ROcp10_67-OMcp10_37*ROcp10_57)+qd[9]*(OMcp10_28*ROcp10_38-OMcp10_38*ROcp10_28);
    OPcp10_210 = OPcp10_27+ROcp10_28*qdd[9]+ROcp10_57*qdd[8]+ROcp10_89*qdd[10]-qd[10]*(OMcp10_19*ROcp10_99-OMcp10_39*
 ROcp10_79)-qd[8]*(OMcp10_17*ROcp10_67-OMcp10_37*ROcp10_47)-qd[9]*(OMcp10_18*ROcp10_38-OMcp10_38*ROcp10_18);
    OPcp10_310 = OPcp10_37+ROcp10_38*qdd[9]+ROcp10_67*qdd[8]+ROcp10_99*qdd[10]+qd[10]*(OMcp10_19*ROcp10_89-OMcp10_29*
 ROcp10_79)+qd[8]*(OMcp10_17*ROcp10_57-OMcp10_27*ROcp10_47)+qd[9]*(OMcp10_18*ROcp10_28-OMcp10_28*ROcp10_18);
    RLcp10_111 = ROcp10_79*s->dpt[3][20];
    RLcp10_211 = ROcp10_89*s->dpt[3][20];
    RLcp10_311 = ROcp10_99*s->dpt[3][20];
    POcp10_111 = RLcp10_111+RLcp10_17+RLcp10_18+q[1];
    POcp10_211 = RLcp10_211+RLcp10_27+RLcp10_28+q[2];
    POcp10_311 = RLcp10_311+RLcp10_37+RLcp10_38+q[3];
    OMcp10_111 = OMcp10_110+ROcp10_410*qd[11];
    OMcp10_211 = OMcp10_210+ROcp10_510*qd[11];
    OMcp10_311 = OMcp10_310+ROcp10_610*qd[11];
    ORcp10_111 = OMcp10_210*RLcp10_311-OMcp10_310*RLcp10_211;
    ORcp10_211 = -(OMcp10_110*RLcp10_311-OMcp10_310*RLcp10_111);
    ORcp10_311 = OMcp10_110*RLcp10_211-OMcp10_210*RLcp10_111;
    VIcp10_111 = ORcp10_111+ORcp10_17+ORcp10_18+qd[1];
    VIcp10_211 = ORcp10_211+ORcp10_27+ORcp10_28+qd[2];
    VIcp10_311 = ORcp10_311+ORcp10_37+ORcp10_38+qd[3];
    OPcp10_111 = OPcp10_110+ROcp10_410*qdd[11]+qd[11]*(OMcp10_210*ROcp10_610-OMcp10_310*ROcp10_510);
    OPcp10_211 = OPcp10_210+ROcp10_510*qdd[11]-qd[11]*(OMcp10_110*ROcp10_610-OMcp10_310*ROcp10_410);
    OPcp10_311 = OPcp10_310+ROcp10_610*qdd[11]+qd[11]*(OMcp10_110*ROcp10_510-OMcp10_210*ROcp10_410);
    ACcp10_111 = qdd[1]+OMcp10_210*ORcp10_311+OMcp10_26*ORcp10_37+OMcp10_27*ORcp10_38-OMcp10_310*ORcp10_211-OMcp10_36*
 ORcp10_27-OMcp10_37*ORcp10_28+OPcp10_210*RLcp10_311+OPcp10_26*RLcp10_37+OPcp10_27*RLcp10_38-OPcp10_310*RLcp10_211-OPcp10_36*
 RLcp10_27-OPcp10_37*RLcp10_28;
    ACcp10_211 = qdd[2]-OMcp10_110*ORcp10_311-OMcp10_16*ORcp10_37-OMcp10_17*ORcp10_38+OMcp10_310*ORcp10_111+OMcp10_36*
 ORcp10_17+OMcp10_37*ORcp10_18-OPcp10_110*RLcp10_311-OPcp10_16*RLcp10_37-OPcp10_17*RLcp10_38+OPcp10_310*RLcp10_111+OPcp10_36*
 RLcp10_17+OPcp10_37*RLcp10_18;
    ACcp10_311 = qdd[3]+OMcp10_110*ORcp10_211+OMcp10_16*ORcp10_27+OMcp10_17*ORcp10_28-OMcp10_210*ORcp10_111-OMcp10_26*
 ORcp10_17-OMcp10_27*ORcp10_18+OPcp10_110*RLcp10_211+OPcp10_16*RLcp10_27+OPcp10_17*RLcp10_28-OPcp10_210*RLcp10_111-OPcp10_26*
 RLcp10_17-OPcp10_27*RLcp10_18;

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_111;
    sens->P[2] = POcp10_211;
    sens->P[3] = POcp10_311;
    sens->R[1][1] = ROcp10_111;
    sens->R[1][2] = ROcp10_211;
    sens->R[1][3] = ROcp10_311;
    sens->R[2][1] = ROcp10_410;
    sens->R[2][2] = ROcp10_510;
    sens->R[2][3] = ROcp10_610;
    sens->R[3][1] = ROcp10_711;
    sens->R[3][2] = ROcp10_811;
    sens->R[3][3] = ROcp10_911;
    sens->V[1] = VIcp10_111;
    sens->V[2] = VIcp10_211;
    sens->V[3] = VIcp10_311;
    sens->OM[1] = OMcp10_111;
    sens->OM[2] = OMcp10_211;
    sens->OM[3] = OMcp10_311;
    sens->A[1] = ACcp10_111;
    sens->A[2] = ACcp10_211;
    sens->A[3] = ACcp10_311;
    sens->OMP[1] = OPcp10_111;
    sens->OMP[2] = OPcp10_211;
    sens->OMP[3] = OPcp10_311;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
// Sensor Kinematics 


    ROcp11_45 = -S4*C5;
    ROcp11_55 = C4*C5;
    ROcp11_75 = S4*S5;
    ROcp11_85 = -C4*S5;
    ROcp11_16 = -(ROcp11_75*S6-C4*C6);
    ROcp11_26 = -(ROcp11_85*S6-S4*C6);
    ROcp11_36 = -C5*S6;
    ROcp11_76 = ROcp11_75*C6+C4*S6;
    ROcp11_86 = ROcp11_85*C6+S4*S6;
    ROcp11_96 = C5*C6;
    OMcp11_15 = qd[5]*C4;
    OMcp11_25 = qd[5]*S4;
    OMcp11_16 = OMcp11_15+ROcp11_45*qd[6];
    OMcp11_26 = OMcp11_25+ROcp11_55*qd[6];
    OMcp11_36 = qd[4]+qd[6]*S5;
    OPcp11_16 = ROcp11_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp11_25*S5-ROcp11_55*qd[4]);
    OPcp11_26 = ROcp11_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp11_15*S5-ROcp11_45*qd[4]);
    OPcp11_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_12_0_3 = = 
 
// Sensor Kinematics 


    ROcp11_412 = ROcp11_45*C12+ROcp11_76*S12;
    ROcp11_512 = ROcp11_55*C12+ROcp11_86*S12;
    ROcp11_612 = ROcp11_96*S12+C12*S5;
    ROcp11_712 = -(ROcp11_45*S12-ROcp11_76*C12);
    ROcp11_812 = -(ROcp11_55*S12-ROcp11_86*C12);
    ROcp11_912 = ROcp11_96*C12-S12*S5;
    RLcp11_112 = ROcp11_16*s->dpt[1][2]+ROcp11_45*s->dpt[2][2]+ROcp11_76*s->dpt[3][2];
    RLcp11_212 = ROcp11_26*s->dpt[1][2]+ROcp11_55*s->dpt[2][2]+ROcp11_86*s->dpt[3][2];
    RLcp11_312 = ROcp11_36*s->dpt[1][2]+ROcp11_96*s->dpt[3][2]+s->dpt[2][2]*S5;
    POcp11_112 = RLcp11_112+q[1];
    POcp11_212 = RLcp11_212+q[2];
    POcp11_312 = RLcp11_312+q[3];
    OMcp11_112 = OMcp11_16+ROcp11_16*qd[12];
    OMcp11_212 = OMcp11_26+ROcp11_26*qd[12];
    OMcp11_312 = OMcp11_36+ROcp11_36*qd[12];
    ORcp11_112 = OMcp11_26*RLcp11_312-OMcp11_36*RLcp11_212;
    ORcp11_212 = -(OMcp11_16*RLcp11_312-OMcp11_36*RLcp11_112);
    ORcp11_312 = OMcp11_16*RLcp11_212-OMcp11_26*RLcp11_112;
    VIcp11_112 = ORcp11_112+qd[1];
    VIcp11_212 = ORcp11_212+qd[2];
    VIcp11_312 = ORcp11_312+qd[3];
    OPcp11_112 = OPcp11_16+ROcp11_16*qdd[12]+qd[12]*(OMcp11_26*ROcp11_36-OMcp11_36*ROcp11_26);
    OPcp11_212 = OPcp11_26+ROcp11_26*qdd[12]-qd[12]*(OMcp11_16*ROcp11_36-OMcp11_36*ROcp11_16);
    OPcp11_312 = OPcp11_36+ROcp11_36*qdd[12]+qd[12]*(OMcp11_16*ROcp11_26-OMcp11_26*ROcp11_16);
    ACcp11_112 = qdd[1]+OMcp11_26*ORcp11_312-OMcp11_36*ORcp11_212+OPcp11_26*RLcp11_312-OPcp11_36*RLcp11_212;
    ACcp11_212 = qdd[2]-OMcp11_16*ORcp11_312+OMcp11_36*ORcp11_112-OPcp11_16*RLcp11_312+OPcp11_36*RLcp11_112;
    ACcp11_312 = qdd[3]+OMcp11_16*ORcp11_212-OMcp11_26*ORcp11_112+OPcp11_16*RLcp11_212-OPcp11_26*RLcp11_112;

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_112;
    sens->P[2] = POcp11_212;
    sens->P[3] = POcp11_312;
    sens->R[1][1] = ROcp11_16;
    sens->R[1][2] = ROcp11_26;
    sens->R[1][3] = ROcp11_36;
    sens->R[2][1] = ROcp11_412;
    sens->R[2][2] = ROcp11_512;
    sens->R[2][3] = ROcp11_612;
    sens->R[3][1] = ROcp11_712;
    sens->R[3][2] = ROcp11_812;
    sens->R[3][3] = ROcp11_912;
    sens->V[1] = VIcp11_112;
    sens->V[2] = VIcp11_212;
    sens->V[3] = VIcp11_312;
    sens->OM[1] = OMcp11_112;
    sens->OM[2] = OMcp11_212;
    sens->OM[3] = OMcp11_312;
    sens->A[1] = ACcp11_112;
    sens->A[2] = ACcp11_212;
    sens->A[3] = ACcp11_312;
    sens->OMP[1] = OPcp11_112;
    sens->OMP[2] = OPcp11_212;
    sens->OMP[3] = OPcp11_312;
 
// 
break;
case 13:
 


// = = Block_1_0_0_13_0_1 = = 
 
// Sensor Kinematics 


    ROcp12_45 = -S4*C5;
    ROcp12_55 = C4*C5;
    ROcp12_75 = S4*S5;
    ROcp12_85 = -C4*S5;
    ROcp12_16 = -(ROcp12_75*S6-C4*C6);
    ROcp12_26 = -(ROcp12_85*S6-S4*C6);
    ROcp12_36 = -C5*S6;
    ROcp12_76 = ROcp12_75*C6+C4*S6;
    ROcp12_86 = ROcp12_85*C6+S4*S6;
    ROcp12_96 = C5*C6;
    OMcp12_15 = qd[5]*C4;
    OMcp12_25 = qd[5]*S4;
    OMcp12_16 = OMcp12_15+ROcp12_45*qd[6];
    OMcp12_26 = OMcp12_25+ROcp12_55*qd[6];
    OMcp12_36 = qd[4]+qd[6]*S5;
    OPcp12_16 = ROcp12_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp12_25*S5-ROcp12_55*qd[4]);
    OPcp12_26 = ROcp12_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp12_15*S5-ROcp12_45*qd[4]);
    OPcp12_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_13_0_3 = = 
 
// Sensor Kinematics 


    ROcp12_412 = ROcp12_45*C12+ROcp12_76*S12;
    ROcp12_512 = ROcp12_55*C12+ROcp12_86*S12;
    ROcp12_612 = ROcp12_96*S12+C12*S5;
    ROcp12_712 = -(ROcp12_45*S12-ROcp12_76*C12);
    ROcp12_812 = -(ROcp12_55*S12-ROcp12_86*C12);
    ROcp12_912 = ROcp12_96*C12-S12*S5;
    ROcp12_113 = ROcp12_16*C13-ROcp12_712*S13;
    ROcp12_213 = ROcp12_26*C13-ROcp12_812*S13;
    ROcp12_313 = ROcp12_36*C13-ROcp12_912*S13;
    ROcp12_713 = ROcp12_16*S13+ROcp12_712*C13;
    ROcp12_813 = ROcp12_26*S13+ROcp12_812*C13;
    ROcp12_913 = ROcp12_36*S13+ROcp12_912*C13;
    RLcp12_112 = ROcp12_16*s->dpt[1][2]+ROcp12_45*s->dpt[2][2]+ROcp12_76*s->dpt[3][2];
    RLcp12_212 = ROcp12_26*s->dpt[1][2]+ROcp12_55*s->dpt[2][2]+ROcp12_86*s->dpt[3][2];
    RLcp12_312 = ROcp12_36*s->dpt[1][2]+ROcp12_96*s->dpt[3][2]+s->dpt[2][2]*S5;
    OMcp12_112 = OMcp12_16+ROcp12_16*qd[12];
    OMcp12_212 = OMcp12_26+ROcp12_26*qd[12];
    OMcp12_312 = OMcp12_36+ROcp12_36*qd[12];
    ORcp12_112 = OMcp12_26*RLcp12_312-OMcp12_36*RLcp12_212;
    ORcp12_212 = -(OMcp12_16*RLcp12_312-OMcp12_36*RLcp12_112);
    ORcp12_312 = OMcp12_16*RLcp12_212-OMcp12_26*RLcp12_112;
    OPcp12_112 = OPcp12_16+ROcp12_16*qdd[12]+qd[12]*(OMcp12_26*ROcp12_36-OMcp12_36*ROcp12_26);
    OPcp12_212 = OPcp12_26+ROcp12_26*qdd[12]-qd[12]*(OMcp12_16*ROcp12_36-OMcp12_36*ROcp12_16);
    OPcp12_312 = OPcp12_36+ROcp12_36*qdd[12]+qd[12]*(OMcp12_16*ROcp12_26-OMcp12_26*ROcp12_16);
    RLcp12_113 = ROcp12_412*s->dpt[2][22];
    RLcp12_213 = ROcp12_512*s->dpt[2][22];
    RLcp12_313 = ROcp12_612*s->dpt[2][22];
    POcp12_113 = RLcp12_112+RLcp12_113+q[1];
    POcp12_213 = RLcp12_212+RLcp12_213+q[2];
    POcp12_313 = RLcp12_312+RLcp12_313+q[3];
    OMcp12_113 = OMcp12_112+ROcp12_412*qd[13];
    OMcp12_213 = OMcp12_212+ROcp12_512*qd[13];
    OMcp12_313 = OMcp12_312+ROcp12_612*qd[13];
    ORcp12_113 = OMcp12_212*RLcp12_313-OMcp12_312*RLcp12_213;
    ORcp12_213 = -(OMcp12_112*RLcp12_313-OMcp12_312*RLcp12_113);
    ORcp12_313 = OMcp12_112*RLcp12_213-OMcp12_212*RLcp12_113;
    VIcp12_113 = ORcp12_112+ORcp12_113+qd[1];
    VIcp12_213 = ORcp12_212+ORcp12_213+qd[2];
    VIcp12_313 = ORcp12_312+ORcp12_313+qd[3];
    OPcp12_113 = OPcp12_112+ROcp12_412*qdd[13]+qd[13]*(OMcp12_212*ROcp12_612-OMcp12_312*ROcp12_512);
    OPcp12_213 = OPcp12_212+ROcp12_512*qdd[13]-qd[13]*(OMcp12_112*ROcp12_612-OMcp12_312*ROcp12_412);
    OPcp12_313 = OPcp12_312+ROcp12_612*qdd[13]+qd[13]*(OMcp12_112*ROcp12_512-OMcp12_212*ROcp12_412);
    ACcp12_113 = qdd[1]+OMcp12_212*ORcp12_313+OMcp12_26*ORcp12_312-OMcp12_312*ORcp12_213-OMcp12_36*ORcp12_212+OPcp12_212*
 RLcp12_313+OPcp12_26*RLcp12_312-OPcp12_312*RLcp12_213-OPcp12_36*RLcp12_212;
    ACcp12_213 = qdd[2]-OMcp12_112*ORcp12_313-OMcp12_16*ORcp12_312+OMcp12_312*ORcp12_113+OMcp12_36*ORcp12_112-OPcp12_112*
 RLcp12_313-OPcp12_16*RLcp12_312+OPcp12_312*RLcp12_113+OPcp12_36*RLcp12_112;
    ACcp12_313 = qdd[3]+OMcp12_112*ORcp12_213+OMcp12_16*ORcp12_212-OMcp12_212*ORcp12_113-OMcp12_26*ORcp12_112+OPcp12_112*
 RLcp12_213+OPcp12_16*RLcp12_212-OPcp12_212*RLcp12_113-OPcp12_26*RLcp12_112;

// = = Block_1_0_0_13_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp12_113;
    sens->P[2] = POcp12_213;
    sens->P[3] = POcp12_313;
    sens->R[1][1] = ROcp12_113;
    sens->R[1][2] = ROcp12_213;
    sens->R[1][3] = ROcp12_313;
    sens->R[2][1] = ROcp12_412;
    sens->R[2][2] = ROcp12_512;
    sens->R[2][3] = ROcp12_612;
    sens->R[3][1] = ROcp12_713;
    sens->R[3][2] = ROcp12_813;
    sens->R[3][3] = ROcp12_913;
    sens->V[1] = VIcp12_113;
    sens->V[2] = VIcp12_213;
    sens->V[3] = VIcp12_313;
    sens->OM[1] = OMcp12_113;
    sens->OM[2] = OMcp12_213;
    sens->OM[3] = OMcp12_313;
    sens->A[1] = ACcp12_113;
    sens->A[2] = ACcp12_213;
    sens->A[3] = ACcp12_313;
    sens->OMP[1] = OPcp12_113;
    sens->OMP[2] = OPcp12_213;
    sens->OMP[3] = OPcp12_313;
 
// 
break;
case 14:
 


// = = Block_1_0_0_14_0_1 = = 
 
// Sensor Kinematics 


    ROcp13_45 = -S4*C5;
    ROcp13_55 = C4*C5;
    ROcp13_75 = S4*S5;
    ROcp13_85 = -C4*S5;
    ROcp13_16 = -(ROcp13_75*S6-C4*C6);
    ROcp13_26 = -(ROcp13_85*S6-S4*C6);
    ROcp13_36 = -C5*S6;
    ROcp13_76 = ROcp13_75*C6+C4*S6;
    ROcp13_86 = ROcp13_85*C6+S4*S6;
    ROcp13_96 = C5*C6;
    OMcp13_15 = qd[5]*C4;
    OMcp13_25 = qd[5]*S4;
    OMcp13_16 = OMcp13_15+ROcp13_45*qd[6];
    OMcp13_26 = OMcp13_25+ROcp13_55*qd[6];
    OMcp13_36 = qd[4]+qd[6]*S5;
    OPcp13_16 = ROcp13_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp13_25*S5-ROcp13_55*qd[4]);
    OPcp13_26 = ROcp13_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp13_15*S5-ROcp13_45*qd[4]);
    OPcp13_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_14_0_3 = = 
 
// Sensor Kinematics 


    ROcp13_412 = ROcp13_45*C12+ROcp13_76*S12;
    ROcp13_512 = ROcp13_55*C12+ROcp13_86*S12;
    ROcp13_612 = ROcp13_96*S12+C12*S5;
    ROcp13_712 = -(ROcp13_45*S12-ROcp13_76*C12);
    ROcp13_812 = -(ROcp13_55*S12-ROcp13_86*C12);
    ROcp13_912 = ROcp13_96*C12-S12*S5;
    ROcp13_113 = ROcp13_16*C13-ROcp13_712*S13;
    ROcp13_213 = ROcp13_26*C13-ROcp13_812*S13;
    ROcp13_313 = ROcp13_36*C13-ROcp13_912*S13;
    ROcp13_713 = ROcp13_16*S13+ROcp13_712*C13;
    ROcp13_813 = ROcp13_26*S13+ROcp13_812*C13;
    ROcp13_913 = ROcp13_36*S13+ROcp13_912*C13;
    ROcp13_414 = ROcp13_412*C14+ROcp13_713*S14;
    ROcp13_514 = ROcp13_512*C14+ROcp13_813*S14;
    ROcp13_614 = ROcp13_612*C14+ROcp13_913*S14;
    ROcp13_714 = -(ROcp13_412*S14-ROcp13_713*C14);
    ROcp13_814 = -(ROcp13_512*S14-ROcp13_813*C14);
    ROcp13_914 = -(ROcp13_612*S14-ROcp13_913*C14);
    RLcp13_112 = ROcp13_16*s->dpt[1][2]+ROcp13_45*s->dpt[2][2]+ROcp13_76*s->dpt[3][2];
    RLcp13_212 = ROcp13_26*s->dpt[1][2]+ROcp13_55*s->dpt[2][2]+ROcp13_86*s->dpt[3][2];
    RLcp13_312 = ROcp13_36*s->dpt[1][2]+ROcp13_96*s->dpt[3][2]+s->dpt[2][2]*S5;
    OMcp13_112 = OMcp13_16+ROcp13_16*qd[12];
    OMcp13_212 = OMcp13_26+ROcp13_26*qd[12];
    OMcp13_312 = OMcp13_36+ROcp13_36*qd[12];
    ORcp13_112 = OMcp13_26*RLcp13_312-OMcp13_36*RLcp13_212;
    ORcp13_212 = -(OMcp13_16*RLcp13_312-OMcp13_36*RLcp13_112);
    ORcp13_312 = OMcp13_16*RLcp13_212-OMcp13_26*RLcp13_112;
    OPcp13_112 = OPcp13_16+ROcp13_16*qdd[12]+qd[12]*(OMcp13_26*ROcp13_36-OMcp13_36*ROcp13_26);
    OPcp13_212 = OPcp13_26+ROcp13_26*qdd[12]-qd[12]*(OMcp13_16*ROcp13_36-OMcp13_36*ROcp13_16);
    OPcp13_312 = OPcp13_36+ROcp13_36*qdd[12]+qd[12]*(OMcp13_16*ROcp13_26-OMcp13_26*ROcp13_16);
    RLcp13_113 = ROcp13_412*s->dpt[2][22];
    RLcp13_213 = ROcp13_512*s->dpt[2][22];
    RLcp13_313 = ROcp13_612*s->dpt[2][22];
    POcp13_113 = RLcp13_112+RLcp13_113+q[1];
    POcp13_213 = RLcp13_212+RLcp13_213+q[2];
    POcp13_313 = RLcp13_312+RLcp13_313+q[3];
    OMcp13_113 = OMcp13_112+ROcp13_412*qd[13];
    OMcp13_213 = OMcp13_212+ROcp13_512*qd[13];
    OMcp13_313 = OMcp13_312+ROcp13_612*qd[13];
    ORcp13_113 = OMcp13_212*RLcp13_313-OMcp13_312*RLcp13_213;
    ORcp13_213 = -(OMcp13_112*RLcp13_313-OMcp13_312*RLcp13_113);
    ORcp13_313 = OMcp13_112*RLcp13_213-OMcp13_212*RLcp13_113;
    VIcp13_113 = ORcp13_112+ORcp13_113+qd[1];
    VIcp13_213 = ORcp13_212+ORcp13_213+qd[2];
    VIcp13_313 = ORcp13_312+ORcp13_313+qd[3];
    ACcp13_113 = qdd[1]+OMcp13_212*ORcp13_313+OMcp13_26*ORcp13_312-OMcp13_312*ORcp13_213-OMcp13_36*ORcp13_212+OPcp13_212*
 RLcp13_313+OPcp13_26*RLcp13_312-OPcp13_312*RLcp13_213-OPcp13_36*RLcp13_212;
    ACcp13_213 = qdd[2]-OMcp13_112*ORcp13_313-OMcp13_16*ORcp13_312+OMcp13_312*ORcp13_113+OMcp13_36*ORcp13_112-OPcp13_112*
 RLcp13_313-OPcp13_16*RLcp13_312+OPcp13_312*RLcp13_113+OPcp13_36*RLcp13_112;
    ACcp13_313 = qdd[3]+OMcp13_112*ORcp13_213+OMcp13_16*ORcp13_212-OMcp13_212*ORcp13_113-OMcp13_26*ORcp13_112+OPcp13_112*
 RLcp13_213+OPcp13_16*RLcp13_212-OPcp13_212*RLcp13_113-OPcp13_26*RLcp13_112;
    OMcp13_114 = OMcp13_113+ROcp13_113*qd[14];
    OMcp13_214 = OMcp13_213+ROcp13_213*qd[14];
    OMcp13_314 = OMcp13_313+ROcp13_313*qd[14];
    OPcp13_114 = OPcp13_112+ROcp13_113*qdd[14]+ROcp13_412*qdd[13]+qd[13]*(OMcp13_212*ROcp13_612-OMcp13_312*ROcp13_512)+
 qd[14]*(OMcp13_213*ROcp13_313-OMcp13_313*ROcp13_213);
    OPcp13_214 = OPcp13_212+ROcp13_213*qdd[14]+ROcp13_512*qdd[13]-qd[13]*(OMcp13_112*ROcp13_612-OMcp13_312*ROcp13_412)-
 qd[14]*(OMcp13_113*ROcp13_313-OMcp13_313*ROcp13_113);
    OPcp13_314 = OPcp13_312+ROcp13_313*qdd[14]+ROcp13_612*qdd[13]+qd[13]*(OMcp13_112*ROcp13_512-OMcp13_212*ROcp13_412)+
 qd[14]*(OMcp13_113*ROcp13_213-OMcp13_213*ROcp13_113);

// = = Block_1_0_0_14_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp13_113;
    sens->P[2] = POcp13_213;
    sens->P[3] = POcp13_313;
    sens->R[1][1] = ROcp13_113;
    sens->R[1][2] = ROcp13_213;
    sens->R[1][3] = ROcp13_313;
    sens->R[2][1] = ROcp13_414;
    sens->R[2][2] = ROcp13_514;
    sens->R[2][3] = ROcp13_614;
    sens->R[3][1] = ROcp13_714;
    sens->R[3][2] = ROcp13_814;
    sens->R[3][3] = ROcp13_914;
    sens->V[1] = VIcp13_113;
    sens->V[2] = VIcp13_213;
    sens->V[3] = VIcp13_313;
    sens->OM[1] = OMcp13_114;
    sens->OM[2] = OMcp13_214;
    sens->OM[3] = OMcp13_314;
    sens->A[1] = ACcp13_113;
    sens->A[2] = ACcp13_213;
    sens->A[3] = ACcp13_313;
    sens->OMP[1] = OPcp13_114;
    sens->OMP[2] = OPcp13_214;
    sens->OMP[3] = OPcp13_314;
 
// 
break;
case 15:
 


// = = Block_1_0_0_15_0_1 = = 
 
// Sensor Kinematics 


    ROcp14_45 = -S4*C5;
    ROcp14_55 = C4*C5;
    ROcp14_75 = S4*S5;
    ROcp14_85 = -C4*S5;
    ROcp14_16 = -(ROcp14_75*S6-C4*C6);
    ROcp14_26 = -(ROcp14_85*S6-S4*C6);
    ROcp14_36 = -C5*S6;
    ROcp14_76 = ROcp14_75*C6+C4*S6;
    ROcp14_86 = ROcp14_85*C6+S4*S6;
    ROcp14_96 = C5*C6;
    OMcp14_15 = qd[5]*C4;
    OMcp14_25 = qd[5]*S4;
    OMcp14_16 = OMcp14_15+ROcp14_45*qd[6];
    OMcp14_26 = OMcp14_25+ROcp14_55*qd[6];
    OMcp14_36 = qd[4]+qd[6]*S5;
    OPcp14_16 = ROcp14_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp14_25*S5-ROcp14_55*qd[4]);
    OPcp14_26 = ROcp14_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp14_15*S5-ROcp14_45*qd[4]);
    OPcp14_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_15_0_3 = = 
 
// Sensor Kinematics 


    ROcp14_412 = ROcp14_45*C12+ROcp14_76*S12;
    ROcp14_512 = ROcp14_55*C12+ROcp14_86*S12;
    ROcp14_612 = ROcp14_96*S12+C12*S5;
    ROcp14_712 = -(ROcp14_45*S12-ROcp14_76*C12);
    ROcp14_812 = -(ROcp14_55*S12-ROcp14_86*C12);
    ROcp14_912 = ROcp14_96*C12-S12*S5;
    ROcp14_113 = ROcp14_16*C13-ROcp14_712*S13;
    ROcp14_213 = ROcp14_26*C13-ROcp14_812*S13;
    ROcp14_313 = ROcp14_36*C13-ROcp14_912*S13;
    ROcp14_713 = ROcp14_16*S13+ROcp14_712*C13;
    ROcp14_813 = ROcp14_26*S13+ROcp14_812*C13;
    ROcp14_913 = ROcp14_36*S13+ROcp14_912*C13;
    ROcp14_414 = ROcp14_412*C14+ROcp14_713*S14;
    ROcp14_514 = ROcp14_512*C14+ROcp14_813*S14;
    ROcp14_614 = ROcp14_612*C14+ROcp14_913*S14;
    ROcp14_714 = -(ROcp14_412*S14-ROcp14_713*C14);
    ROcp14_814 = -(ROcp14_512*S14-ROcp14_813*C14);
    ROcp14_914 = -(ROcp14_612*S14-ROcp14_913*C14);
    ROcp14_115 = ROcp14_113*C15+ROcp14_414*S15;
    ROcp14_215 = ROcp14_213*C15+ROcp14_514*S15;
    ROcp14_315 = ROcp14_313*C15+ROcp14_614*S15;
    ROcp14_415 = -(ROcp14_113*S15-ROcp14_414*C15);
    ROcp14_515 = -(ROcp14_213*S15-ROcp14_514*C15);
    ROcp14_615 = -(ROcp14_313*S15-ROcp14_614*C15);
    RLcp14_112 = ROcp14_16*s->dpt[1][2]+ROcp14_45*s->dpt[2][2]+ROcp14_76*s->dpt[3][2];
    RLcp14_212 = ROcp14_26*s->dpt[1][2]+ROcp14_55*s->dpt[2][2]+ROcp14_86*s->dpt[3][2];
    RLcp14_312 = ROcp14_36*s->dpt[1][2]+ROcp14_96*s->dpt[3][2]+s->dpt[2][2]*S5;
    OMcp14_112 = OMcp14_16+ROcp14_16*qd[12];
    OMcp14_212 = OMcp14_26+ROcp14_26*qd[12];
    OMcp14_312 = OMcp14_36+ROcp14_36*qd[12];
    ORcp14_112 = OMcp14_26*RLcp14_312-OMcp14_36*RLcp14_212;
    ORcp14_212 = -(OMcp14_16*RLcp14_312-OMcp14_36*RLcp14_112);
    ORcp14_312 = OMcp14_16*RLcp14_212-OMcp14_26*RLcp14_112;
    OPcp14_112 = OPcp14_16+ROcp14_16*qdd[12]+qd[12]*(OMcp14_26*ROcp14_36-OMcp14_36*ROcp14_26);
    OPcp14_212 = OPcp14_26+ROcp14_26*qdd[12]-qd[12]*(OMcp14_16*ROcp14_36-OMcp14_36*ROcp14_16);
    OPcp14_312 = OPcp14_36+ROcp14_36*qdd[12]+qd[12]*(OMcp14_16*ROcp14_26-OMcp14_26*ROcp14_16);
    RLcp14_113 = ROcp14_412*s->dpt[2][22];
    RLcp14_213 = ROcp14_512*s->dpt[2][22];
    RLcp14_313 = ROcp14_612*s->dpt[2][22];
    POcp14_113 = RLcp14_112+RLcp14_113+q[1];
    POcp14_213 = RLcp14_212+RLcp14_213+q[2];
    POcp14_313 = RLcp14_312+RLcp14_313+q[3];
    OMcp14_113 = OMcp14_112+ROcp14_412*qd[13];
    OMcp14_213 = OMcp14_212+ROcp14_512*qd[13];
    OMcp14_313 = OMcp14_312+ROcp14_612*qd[13];
    ORcp14_113 = OMcp14_212*RLcp14_313-OMcp14_312*RLcp14_213;
    ORcp14_213 = -(OMcp14_112*RLcp14_313-OMcp14_312*RLcp14_113);
    ORcp14_313 = OMcp14_112*RLcp14_213-OMcp14_212*RLcp14_113;
    VIcp14_113 = ORcp14_112+ORcp14_113+qd[1];
    VIcp14_213 = ORcp14_212+ORcp14_213+qd[2];
    VIcp14_313 = ORcp14_312+ORcp14_313+qd[3];
    ACcp14_113 = qdd[1]+OMcp14_212*ORcp14_313+OMcp14_26*ORcp14_312-OMcp14_312*ORcp14_213-OMcp14_36*ORcp14_212+OPcp14_212*
 RLcp14_313+OPcp14_26*RLcp14_312-OPcp14_312*RLcp14_213-OPcp14_36*RLcp14_212;
    ACcp14_213 = qdd[2]-OMcp14_112*ORcp14_313-OMcp14_16*ORcp14_312+OMcp14_312*ORcp14_113+OMcp14_36*ORcp14_112-OPcp14_112*
 RLcp14_313-OPcp14_16*RLcp14_312+OPcp14_312*RLcp14_113+OPcp14_36*RLcp14_112;
    ACcp14_313 = qdd[3]+OMcp14_112*ORcp14_213+OMcp14_16*ORcp14_212-OMcp14_212*ORcp14_113-OMcp14_26*ORcp14_112+OPcp14_112*
 RLcp14_213+OPcp14_16*RLcp14_212-OPcp14_212*RLcp14_113-OPcp14_26*RLcp14_112;
    OMcp14_114 = OMcp14_113+ROcp14_113*qd[14];
    OMcp14_214 = OMcp14_213+ROcp14_213*qd[14];
    OMcp14_314 = OMcp14_313+ROcp14_313*qd[14];
    OMcp14_115 = OMcp14_114+ROcp14_714*qd[15];
    OMcp14_215 = OMcp14_214+ROcp14_814*qd[15];
    OMcp14_315 = OMcp14_314+ROcp14_914*qd[15];
    OPcp14_115 = OPcp14_112+ROcp14_113*qdd[14]+ROcp14_412*qdd[13]+ROcp14_714*qdd[15]+qd[13]*(OMcp14_212*ROcp14_612-
 OMcp14_312*ROcp14_512)+qd[14]*(OMcp14_213*ROcp14_313-OMcp14_313*ROcp14_213)+qd[15]*(OMcp14_214*ROcp14_914-OMcp14_314*
 ROcp14_814);
    OPcp14_215 = OPcp14_212+ROcp14_213*qdd[14]+ROcp14_512*qdd[13]+ROcp14_814*qdd[15]-qd[13]*(OMcp14_112*ROcp14_612-
 OMcp14_312*ROcp14_412)-qd[14]*(OMcp14_113*ROcp14_313-OMcp14_313*ROcp14_113)-qd[15]*(OMcp14_114*ROcp14_914-OMcp14_314*
 ROcp14_714);
    OPcp14_315 = OPcp14_312+ROcp14_313*qdd[14]+ROcp14_612*qdd[13]+ROcp14_914*qdd[15]+qd[13]*(OMcp14_112*ROcp14_512-
 OMcp14_212*ROcp14_412)+qd[14]*(OMcp14_113*ROcp14_213-OMcp14_213*ROcp14_113)+qd[15]*(OMcp14_114*ROcp14_814-OMcp14_214*
 ROcp14_714);

// = = Block_1_0_0_15_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp14_113;
    sens->P[2] = POcp14_213;
    sens->P[3] = POcp14_313;
    sens->R[1][1] = ROcp14_115;
    sens->R[1][2] = ROcp14_215;
    sens->R[1][3] = ROcp14_315;
    sens->R[2][1] = ROcp14_415;
    sens->R[2][2] = ROcp14_515;
    sens->R[2][3] = ROcp14_615;
    sens->R[3][1] = ROcp14_714;
    sens->R[3][2] = ROcp14_814;
    sens->R[3][3] = ROcp14_914;
    sens->V[1] = VIcp14_113;
    sens->V[2] = VIcp14_213;
    sens->V[3] = VIcp14_313;
    sens->OM[1] = OMcp14_115;
    sens->OM[2] = OMcp14_215;
    sens->OM[3] = OMcp14_315;
    sens->A[1] = ACcp14_113;
    sens->A[2] = ACcp14_213;
    sens->A[3] = ACcp14_313;
    sens->OMP[1] = OPcp14_115;
    sens->OMP[2] = OPcp14_215;
    sens->OMP[3] = OPcp14_315;
 
// 
break;
case 16:
 


// = = Block_1_0_0_16_0_1 = = 
 
// Sensor Kinematics 


    ROcp15_45 = -S4*C5;
    ROcp15_55 = C4*C5;
    ROcp15_75 = S4*S5;
    ROcp15_85 = -C4*S5;
    ROcp15_16 = -(ROcp15_75*S6-C4*C6);
    ROcp15_26 = -(ROcp15_85*S6-S4*C6);
    ROcp15_36 = -C5*S6;
    ROcp15_76 = ROcp15_75*C6+C4*S6;
    ROcp15_86 = ROcp15_85*C6+S4*S6;
    ROcp15_96 = C5*C6;
    OMcp15_15 = qd[5]*C4;
    OMcp15_25 = qd[5]*S4;
    OMcp15_16 = OMcp15_15+ROcp15_45*qd[6];
    OMcp15_26 = OMcp15_25+ROcp15_55*qd[6];
    OMcp15_36 = qd[4]+qd[6]*S5;
    OPcp15_16 = ROcp15_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp15_25*S5-ROcp15_55*qd[4]);
    OPcp15_26 = ROcp15_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp15_15*S5-ROcp15_45*qd[4]);
    OPcp15_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_16_0_3 = = 
 
// Sensor Kinematics 


    ROcp15_412 = ROcp15_45*C12+ROcp15_76*S12;
    ROcp15_512 = ROcp15_55*C12+ROcp15_86*S12;
    ROcp15_612 = ROcp15_96*S12+C12*S5;
    ROcp15_712 = -(ROcp15_45*S12-ROcp15_76*C12);
    ROcp15_812 = -(ROcp15_55*S12-ROcp15_86*C12);
    ROcp15_912 = ROcp15_96*C12-S12*S5;
    ROcp15_113 = ROcp15_16*C13-ROcp15_712*S13;
    ROcp15_213 = ROcp15_26*C13-ROcp15_812*S13;
    ROcp15_313 = ROcp15_36*C13-ROcp15_912*S13;
    ROcp15_713 = ROcp15_16*S13+ROcp15_712*C13;
    ROcp15_813 = ROcp15_26*S13+ROcp15_812*C13;
    ROcp15_913 = ROcp15_36*S13+ROcp15_912*C13;
    ROcp15_414 = ROcp15_412*C14+ROcp15_713*S14;
    ROcp15_514 = ROcp15_512*C14+ROcp15_813*S14;
    ROcp15_614 = ROcp15_612*C14+ROcp15_913*S14;
    ROcp15_714 = -(ROcp15_412*S14-ROcp15_713*C14);
    ROcp15_814 = -(ROcp15_512*S14-ROcp15_813*C14);
    ROcp15_914 = -(ROcp15_612*S14-ROcp15_913*C14);
    ROcp15_115 = ROcp15_113*C15+ROcp15_414*S15;
    ROcp15_215 = ROcp15_213*C15+ROcp15_514*S15;
    ROcp15_315 = ROcp15_313*C15+ROcp15_614*S15;
    ROcp15_415 = -(ROcp15_113*S15-ROcp15_414*C15);
    ROcp15_515 = -(ROcp15_213*S15-ROcp15_514*C15);
    ROcp15_615 = -(ROcp15_313*S15-ROcp15_614*C15);
    ROcp15_116 = ROcp15_115*C16-ROcp15_714*S16;
    ROcp15_216 = ROcp15_215*C16-ROcp15_814*S16;
    ROcp15_316 = ROcp15_315*C16-ROcp15_914*S16;
    ROcp15_716 = ROcp15_115*S16+ROcp15_714*C16;
    ROcp15_816 = ROcp15_215*S16+ROcp15_814*C16;
    ROcp15_916 = ROcp15_315*S16+ROcp15_914*C16;
    RLcp15_112 = ROcp15_16*s->dpt[1][2]+ROcp15_45*s->dpt[2][2]+ROcp15_76*s->dpt[3][2];
    RLcp15_212 = ROcp15_26*s->dpt[1][2]+ROcp15_55*s->dpt[2][2]+ROcp15_86*s->dpt[3][2];
    RLcp15_312 = ROcp15_36*s->dpt[1][2]+ROcp15_96*s->dpt[3][2]+s->dpt[2][2]*S5;
    OMcp15_112 = OMcp15_16+ROcp15_16*qd[12];
    OMcp15_212 = OMcp15_26+ROcp15_26*qd[12];
    OMcp15_312 = OMcp15_36+ROcp15_36*qd[12];
    ORcp15_112 = OMcp15_26*RLcp15_312-OMcp15_36*RLcp15_212;
    ORcp15_212 = -(OMcp15_16*RLcp15_312-OMcp15_36*RLcp15_112);
    ORcp15_312 = OMcp15_16*RLcp15_212-OMcp15_26*RLcp15_112;
    OPcp15_112 = OPcp15_16+ROcp15_16*qdd[12]+qd[12]*(OMcp15_26*ROcp15_36-OMcp15_36*ROcp15_26);
    OPcp15_212 = OPcp15_26+ROcp15_26*qdd[12]-qd[12]*(OMcp15_16*ROcp15_36-OMcp15_36*ROcp15_16);
    OPcp15_312 = OPcp15_36+ROcp15_36*qdd[12]+qd[12]*(OMcp15_16*ROcp15_26-OMcp15_26*ROcp15_16);
    RLcp15_113 = ROcp15_412*s->dpt[2][22];
    RLcp15_213 = ROcp15_512*s->dpt[2][22];
    RLcp15_313 = ROcp15_612*s->dpt[2][22];
    OMcp15_113 = OMcp15_112+ROcp15_412*qd[13];
    OMcp15_213 = OMcp15_212+ROcp15_512*qd[13];
    OMcp15_313 = OMcp15_312+ROcp15_612*qd[13];
    ORcp15_113 = OMcp15_212*RLcp15_313-OMcp15_312*RLcp15_213;
    ORcp15_213 = -(OMcp15_112*RLcp15_313-OMcp15_312*RLcp15_113);
    ORcp15_313 = OMcp15_112*RLcp15_213-OMcp15_212*RLcp15_113;
    OMcp15_114 = OMcp15_113+ROcp15_113*qd[14];
    OMcp15_214 = OMcp15_213+ROcp15_213*qd[14];
    OMcp15_314 = OMcp15_313+ROcp15_313*qd[14];
    OMcp15_115 = OMcp15_114+ROcp15_714*qd[15];
    OMcp15_215 = OMcp15_214+ROcp15_814*qd[15];
    OMcp15_315 = OMcp15_314+ROcp15_914*qd[15];
    OPcp15_115 = OPcp15_112+ROcp15_113*qdd[14]+ROcp15_412*qdd[13]+ROcp15_714*qdd[15]+qd[13]*(OMcp15_212*ROcp15_612-
 OMcp15_312*ROcp15_512)+qd[14]*(OMcp15_213*ROcp15_313-OMcp15_313*ROcp15_213)+qd[15]*(OMcp15_214*ROcp15_914-OMcp15_314*
 ROcp15_814);
    OPcp15_215 = OPcp15_212+ROcp15_213*qdd[14]+ROcp15_512*qdd[13]+ROcp15_814*qdd[15]-qd[13]*(OMcp15_112*ROcp15_612-
 OMcp15_312*ROcp15_412)-qd[14]*(OMcp15_113*ROcp15_313-OMcp15_313*ROcp15_113)-qd[15]*(OMcp15_114*ROcp15_914-OMcp15_314*
 ROcp15_714);
    OPcp15_315 = OPcp15_312+ROcp15_313*qdd[14]+ROcp15_612*qdd[13]+ROcp15_914*qdd[15]+qd[13]*(OMcp15_112*ROcp15_512-
 OMcp15_212*ROcp15_412)+qd[14]*(OMcp15_113*ROcp15_213-OMcp15_213*ROcp15_113)+qd[15]*(OMcp15_114*ROcp15_814-OMcp15_214*
 ROcp15_714);
    RLcp15_116 = ROcp15_714*s->dpt[3][25];
    RLcp15_216 = ROcp15_814*s->dpt[3][25];
    RLcp15_316 = ROcp15_914*s->dpt[3][25];
    POcp15_116 = RLcp15_112+RLcp15_113+RLcp15_116+q[1];
    POcp15_216 = RLcp15_212+RLcp15_213+RLcp15_216+q[2];
    POcp15_316 = RLcp15_312+RLcp15_313+RLcp15_316+q[3];
    OMcp15_116 = OMcp15_115+ROcp15_415*qd[16];
    OMcp15_216 = OMcp15_215+ROcp15_515*qd[16];
    OMcp15_316 = OMcp15_315+ROcp15_615*qd[16];
    ORcp15_116 = OMcp15_215*RLcp15_316-OMcp15_315*RLcp15_216;
    ORcp15_216 = -(OMcp15_115*RLcp15_316-OMcp15_315*RLcp15_116);
    ORcp15_316 = OMcp15_115*RLcp15_216-OMcp15_215*RLcp15_116;
    VIcp15_116 = ORcp15_112+ORcp15_113+ORcp15_116+qd[1];
    VIcp15_216 = ORcp15_212+ORcp15_213+ORcp15_216+qd[2];
    VIcp15_316 = ORcp15_312+ORcp15_313+ORcp15_316+qd[3];
    OPcp15_116 = OPcp15_115+ROcp15_415*qdd[16]+qd[16]*(OMcp15_215*ROcp15_615-OMcp15_315*ROcp15_515);
    OPcp15_216 = OPcp15_215+ROcp15_515*qdd[16]-qd[16]*(OMcp15_115*ROcp15_615-OMcp15_315*ROcp15_415);
    OPcp15_316 = OPcp15_315+ROcp15_615*qdd[16]+qd[16]*(OMcp15_115*ROcp15_515-OMcp15_215*ROcp15_415);
    ACcp15_116 = qdd[1]+OMcp15_212*ORcp15_313+OMcp15_215*ORcp15_316+OMcp15_26*ORcp15_312-OMcp15_312*ORcp15_213-OMcp15_315*
 ORcp15_216-OMcp15_36*ORcp15_212+OPcp15_212*RLcp15_313+OPcp15_215*RLcp15_316+OPcp15_26*RLcp15_312-OPcp15_312*RLcp15_213-
 OPcp15_315*RLcp15_216-OPcp15_36*RLcp15_212;
    ACcp15_216 = qdd[2]-OMcp15_112*ORcp15_313-OMcp15_115*ORcp15_316-OMcp15_16*ORcp15_312+OMcp15_312*ORcp15_113+OMcp15_315*
 ORcp15_116+OMcp15_36*ORcp15_112-OPcp15_112*RLcp15_313-OPcp15_115*RLcp15_316-OPcp15_16*RLcp15_312+OPcp15_312*RLcp15_113+
 OPcp15_315*RLcp15_116+OPcp15_36*RLcp15_112;
    ACcp15_316 = qdd[3]+OMcp15_112*ORcp15_213+OMcp15_115*ORcp15_216+OMcp15_16*ORcp15_212-OMcp15_212*ORcp15_113-OMcp15_215*
 ORcp15_116-OMcp15_26*ORcp15_112+OPcp15_112*RLcp15_213+OPcp15_115*RLcp15_216+OPcp15_16*RLcp15_212-OPcp15_212*RLcp15_113-
 OPcp15_215*RLcp15_116-OPcp15_26*RLcp15_112;

// = = Block_1_0_0_16_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp15_116;
    sens->P[2] = POcp15_216;
    sens->P[3] = POcp15_316;
    sens->R[1][1] = ROcp15_116;
    sens->R[1][2] = ROcp15_216;
    sens->R[1][3] = ROcp15_316;
    sens->R[2][1] = ROcp15_415;
    sens->R[2][2] = ROcp15_515;
    sens->R[2][3] = ROcp15_615;
    sens->R[3][1] = ROcp15_716;
    sens->R[3][2] = ROcp15_816;
    sens->R[3][3] = ROcp15_916;
    sens->V[1] = VIcp15_116;
    sens->V[2] = VIcp15_216;
    sens->V[3] = VIcp15_316;
    sens->OM[1] = OMcp15_116;
    sens->OM[2] = OMcp15_216;
    sens->OM[3] = OMcp15_316;
    sens->A[1] = ACcp15_116;
    sens->A[2] = ACcp15_216;
    sens->A[3] = ACcp15_316;
    sens->OMP[1] = OPcp15_116;
    sens->OMP[2] = OPcp15_216;
    sens->OMP[3] = OPcp15_316;
 
// 
break;
case 17:
 


// = = Block_1_0_0_17_0_1 = = 
 
// Sensor Kinematics 


    ROcp16_45 = -S4*C5;
    ROcp16_55 = C4*C5;
    ROcp16_75 = S4*S5;
    ROcp16_85 = -C4*S5;
    ROcp16_16 = -(ROcp16_75*S6-C4*C6);
    ROcp16_26 = -(ROcp16_85*S6-S4*C6);
    ROcp16_36 = -C5*S6;
    ROcp16_76 = ROcp16_75*C6+C4*S6;
    ROcp16_86 = ROcp16_85*C6+S4*S6;
    ROcp16_96 = C5*C6;
    OMcp16_15 = qd[5]*C4;
    OMcp16_25 = qd[5]*S4;
    OMcp16_16 = OMcp16_15+ROcp16_45*qd[6];
    OMcp16_26 = OMcp16_25+ROcp16_55*qd[6];
    OMcp16_36 = qd[4]+qd[6]*S5;
    OPcp16_16 = ROcp16_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp16_25*S5-ROcp16_55*qd[4]);
    OPcp16_26 = ROcp16_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp16_15*S5-ROcp16_45*qd[4]);
    OPcp16_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_17_0_4 = = 
 
// Sensor Kinematics 


    ROcp16_417 = ROcp16_45*C17+ROcp16_76*S17;
    ROcp16_517 = ROcp16_55*C17+ROcp16_86*S17;
    ROcp16_617 = ROcp16_96*S17+C17*S5;
    ROcp16_717 = -(ROcp16_45*S17-ROcp16_76*C17);
    ROcp16_817 = -(ROcp16_55*S17-ROcp16_86*C17);
    ROcp16_917 = ROcp16_96*C17-S17*S5;
    RLcp16_117 = ROcp16_45*s->dpt[2][3];
    RLcp16_217 = ROcp16_55*s->dpt[2][3];
    RLcp16_317 = s->dpt[2][3]*S5;
    POcp16_117 = RLcp16_117+q[1];
    POcp16_217 = RLcp16_217+q[2];
    POcp16_317 = RLcp16_317+q[3];
    OMcp16_117 = OMcp16_16+ROcp16_16*qd[17];
    OMcp16_217 = OMcp16_26+ROcp16_26*qd[17];
    OMcp16_317 = OMcp16_36+ROcp16_36*qd[17];
    ORcp16_117 = OMcp16_26*RLcp16_317-OMcp16_36*RLcp16_217;
    ORcp16_217 = -(OMcp16_16*RLcp16_317-OMcp16_36*RLcp16_117);
    ORcp16_317 = OMcp16_16*RLcp16_217-OMcp16_26*RLcp16_117;
    VIcp16_117 = ORcp16_117+qd[1];
    VIcp16_217 = ORcp16_217+qd[2];
    VIcp16_317 = ORcp16_317+qd[3];
    OPcp16_117 = OPcp16_16+ROcp16_16*qdd[17]+qd[17]*(OMcp16_26*ROcp16_36-OMcp16_36*ROcp16_26);
    OPcp16_217 = OPcp16_26+ROcp16_26*qdd[17]-qd[17]*(OMcp16_16*ROcp16_36-OMcp16_36*ROcp16_16);
    OPcp16_317 = OPcp16_36+ROcp16_36*qdd[17]+qd[17]*(OMcp16_16*ROcp16_26-OMcp16_26*ROcp16_16);
    ACcp16_117 = qdd[1]+OMcp16_26*ORcp16_317-OMcp16_36*ORcp16_217+OPcp16_26*RLcp16_317-OPcp16_36*RLcp16_217;
    ACcp16_217 = qdd[2]-OMcp16_16*ORcp16_317+OMcp16_36*ORcp16_117-OPcp16_16*RLcp16_317+OPcp16_36*RLcp16_117;
    ACcp16_317 = qdd[3]+OMcp16_16*ORcp16_217-OMcp16_26*ORcp16_117+OPcp16_16*RLcp16_217-OPcp16_26*RLcp16_117;

// = = Block_1_0_0_17_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp16_117;
    sens->P[2] = POcp16_217;
    sens->P[3] = POcp16_317;
    sens->R[1][1] = ROcp16_16;
    sens->R[1][2] = ROcp16_26;
    sens->R[1][3] = ROcp16_36;
    sens->R[2][1] = ROcp16_417;
    sens->R[2][2] = ROcp16_517;
    sens->R[2][3] = ROcp16_617;
    sens->R[3][1] = ROcp16_717;
    sens->R[3][2] = ROcp16_817;
    sens->R[3][3] = ROcp16_917;
    sens->V[1] = VIcp16_117;
    sens->V[2] = VIcp16_217;
    sens->V[3] = VIcp16_317;
    sens->OM[1] = OMcp16_117;
    sens->OM[2] = OMcp16_217;
    sens->OM[3] = OMcp16_317;
    sens->A[1] = ACcp16_117;
    sens->A[2] = ACcp16_217;
    sens->A[3] = ACcp16_317;
    sens->OMP[1] = OPcp16_117;
    sens->OMP[2] = OPcp16_217;
    sens->OMP[3] = OPcp16_317;
 
// 
break;
case 18:
 


// = = Block_1_0_0_18_0_1 = = 
 
// Sensor Kinematics 


    ROcp17_45 = -S4*C5;
    ROcp17_55 = C4*C5;
    ROcp17_75 = S4*S5;
    ROcp17_85 = -C4*S5;
    ROcp17_16 = -(ROcp17_75*S6-C4*C6);
    ROcp17_26 = -(ROcp17_85*S6-S4*C6);
    ROcp17_36 = -C5*S6;
    ROcp17_76 = ROcp17_75*C6+C4*S6;
    ROcp17_86 = ROcp17_85*C6+S4*S6;
    ROcp17_96 = C5*C6;
    OMcp17_15 = qd[5]*C4;
    OMcp17_25 = qd[5]*S4;
    OMcp17_16 = OMcp17_15+ROcp17_45*qd[6];
    OMcp17_26 = OMcp17_25+ROcp17_55*qd[6];
    OMcp17_36 = qd[4]+qd[6]*S5;
    OPcp17_16 = ROcp17_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp17_25*S5-ROcp17_55*qd[4]);
    OPcp17_26 = ROcp17_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp17_15*S5-ROcp17_45*qd[4]);
    OPcp17_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_18_0_5 = = 
 
// Sensor Kinematics 


    ROcp17_418 = ROcp17_45*C18+ROcp17_76*S18;
    ROcp17_518 = ROcp17_55*C18+ROcp17_86*S18;
    ROcp17_618 = ROcp17_96*S18+C18*S5;
    ROcp17_718 = -(ROcp17_45*S18-ROcp17_76*C18);
    ROcp17_818 = -(ROcp17_55*S18-ROcp17_86*C18);
    ROcp17_918 = ROcp17_96*C18-S18*S5;
    RLcp17_118 = ROcp17_45*s->dpt[2][4];
    RLcp17_218 = ROcp17_55*s->dpt[2][4];
    RLcp17_318 = s->dpt[2][4]*S5;
    POcp17_118 = RLcp17_118+q[1];
    POcp17_218 = RLcp17_218+q[2];
    POcp17_318 = RLcp17_318+q[3];
    OMcp17_118 = OMcp17_16+ROcp17_16*qd[18];
    OMcp17_218 = OMcp17_26+ROcp17_26*qd[18];
    OMcp17_318 = OMcp17_36+ROcp17_36*qd[18];
    ORcp17_118 = OMcp17_26*RLcp17_318-OMcp17_36*RLcp17_218;
    ORcp17_218 = -(OMcp17_16*RLcp17_318-OMcp17_36*RLcp17_118);
    ORcp17_318 = OMcp17_16*RLcp17_218-OMcp17_26*RLcp17_118;
    VIcp17_118 = ORcp17_118+qd[1];
    VIcp17_218 = ORcp17_218+qd[2];
    VIcp17_318 = ORcp17_318+qd[3];
    OPcp17_118 = OPcp17_16+ROcp17_16*qdd[18]+qd[18]*(OMcp17_26*ROcp17_36-OMcp17_36*ROcp17_26);
    OPcp17_218 = OPcp17_26+ROcp17_26*qdd[18]-qd[18]*(OMcp17_16*ROcp17_36-OMcp17_36*ROcp17_16);
    OPcp17_318 = OPcp17_36+ROcp17_36*qdd[18]+qd[18]*(OMcp17_16*ROcp17_26-OMcp17_26*ROcp17_16);
    ACcp17_118 = qdd[1]+OMcp17_26*ORcp17_318-OMcp17_36*ORcp17_218+OPcp17_26*RLcp17_318-OPcp17_36*RLcp17_218;
    ACcp17_218 = qdd[2]-OMcp17_16*ORcp17_318+OMcp17_36*ORcp17_118-OPcp17_16*RLcp17_318+OPcp17_36*RLcp17_118;
    ACcp17_318 = qdd[3]+OMcp17_16*ORcp17_218-OMcp17_26*ORcp17_118+OPcp17_16*RLcp17_218-OPcp17_26*RLcp17_118;

// = = Block_1_0_0_18_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp17_118;
    sens->P[2] = POcp17_218;
    sens->P[3] = POcp17_318;
    sens->R[1][1] = ROcp17_16;
    sens->R[1][2] = ROcp17_26;
    sens->R[1][3] = ROcp17_36;
    sens->R[2][1] = ROcp17_418;
    sens->R[2][2] = ROcp17_518;
    sens->R[2][3] = ROcp17_618;
    sens->R[3][1] = ROcp17_718;
    sens->R[3][2] = ROcp17_818;
    sens->R[3][3] = ROcp17_918;
    sens->V[1] = VIcp17_118;
    sens->V[2] = VIcp17_218;
    sens->V[3] = VIcp17_318;
    sens->OM[1] = OMcp17_118;
    sens->OM[2] = OMcp17_218;
    sens->OM[3] = OMcp17_318;
    sens->A[1] = ACcp17_118;
    sens->A[2] = ACcp17_218;
    sens->A[3] = ACcp17_318;
    sens->OMP[1] = OPcp17_118;
    sens->OMP[2] = OPcp17_218;
    sens->OMP[3] = OPcp17_318;
 
// 
break;
case 19:
 


// = = Block_1_0_0_19_0_1 = = 
 
// Sensor Kinematics 


    ROcp18_45 = -S4*C5;
    ROcp18_55 = C4*C5;
    ROcp18_75 = S4*S5;
    ROcp18_85 = -C4*S5;
    ROcp18_16 = -(ROcp18_75*S6-C4*C6);
    ROcp18_26 = -(ROcp18_85*S6-S4*C6);
    ROcp18_36 = -C5*S6;
    ROcp18_76 = ROcp18_75*C6+C4*S6;
    ROcp18_86 = ROcp18_85*C6+S4*S6;
    ROcp18_96 = C5*C6;
    OMcp18_15 = qd[5]*C4;
    OMcp18_25 = qd[5]*S4;
    OMcp18_16 = OMcp18_15+ROcp18_45*qd[6];
    OMcp18_26 = OMcp18_25+ROcp18_55*qd[6];
    OMcp18_36 = qd[4]+qd[6]*S5;
    OPcp18_16 = ROcp18_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp18_25*S5-ROcp18_55*qd[4]);
    OPcp18_26 = ROcp18_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp18_15*S5-ROcp18_45*qd[4]);
    OPcp18_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_19_0_6 = = 
 
// Sensor Kinematics 


    ROcp18_419 = ROcp18_45*C19+ROcp18_76*S19;
    ROcp18_519 = ROcp18_55*C19+ROcp18_86*S19;
    ROcp18_619 = ROcp18_96*S19+C19*S5;
    ROcp18_719 = -(ROcp18_45*S19-ROcp18_76*C19);
    ROcp18_819 = -(ROcp18_55*S19-ROcp18_86*C19);
    ROcp18_919 = ROcp18_96*C19-S19*S5;
    RLcp18_119 = ROcp18_16*s->dpt[1][5]+ROcp18_45*s->dpt[2][5];
    RLcp18_219 = ROcp18_26*s->dpt[1][5]+ROcp18_55*s->dpt[2][5];
    RLcp18_319 = ROcp18_36*s->dpt[1][5]+s->dpt[2][5]*S5;
    POcp18_119 = RLcp18_119+q[1];
    POcp18_219 = RLcp18_219+q[2];
    POcp18_319 = RLcp18_319+q[3];
    OMcp18_119 = OMcp18_16+ROcp18_16*qd[19];
    OMcp18_219 = OMcp18_26+ROcp18_26*qd[19];
    OMcp18_319 = OMcp18_36+ROcp18_36*qd[19];
    ORcp18_119 = OMcp18_26*RLcp18_319-OMcp18_36*RLcp18_219;
    ORcp18_219 = -(OMcp18_16*RLcp18_319-OMcp18_36*RLcp18_119);
    ORcp18_319 = OMcp18_16*RLcp18_219-OMcp18_26*RLcp18_119;
    VIcp18_119 = ORcp18_119+qd[1];
    VIcp18_219 = ORcp18_219+qd[2];
    VIcp18_319 = ORcp18_319+qd[3];
    OPcp18_119 = OPcp18_16+ROcp18_16*qdd[19]+qd[19]*(OMcp18_26*ROcp18_36-OMcp18_36*ROcp18_26);
    OPcp18_219 = OPcp18_26+ROcp18_26*qdd[19]-qd[19]*(OMcp18_16*ROcp18_36-OMcp18_36*ROcp18_16);
    OPcp18_319 = OPcp18_36+ROcp18_36*qdd[19]+qd[19]*(OMcp18_16*ROcp18_26-OMcp18_26*ROcp18_16);
    ACcp18_119 = qdd[1]+OMcp18_26*ORcp18_319-OMcp18_36*ORcp18_219+OPcp18_26*RLcp18_319-OPcp18_36*RLcp18_219;
    ACcp18_219 = qdd[2]-OMcp18_16*ORcp18_319+OMcp18_36*ORcp18_119-OPcp18_16*RLcp18_319+OPcp18_36*RLcp18_119;
    ACcp18_319 = qdd[3]+OMcp18_16*ORcp18_219-OMcp18_26*ORcp18_119+OPcp18_16*RLcp18_219-OPcp18_26*RLcp18_119;

// = = Block_1_0_0_19_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp18_119;
    sens->P[2] = POcp18_219;
    sens->P[3] = POcp18_319;
    sens->R[1][1] = ROcp18_16;
    sens->R[1][2] = ROcp18_26;
    sens->R[1][3] = ROcp18_36;
    sens->R[2][1] = ROcp18_419;
    sens->R[2][2] = ROcp18_519;
    sens->R[2][3] = ROcp18_619;
    sens->R[3][1] = ROcp18_719;
    sens->R[3][2] = ROcp18_819;
    sens->R[3][3] = ROcp18_919;
    sens->V[1] = VIcp18_119;
    sens->V[2] = VIcp18_219;
    sens->V[3] = VIcp18_319;
    sens->OM[1] = OMcp18_119;
    sens->OM[2] = OMcp18_219;
    sens->OM[3] = OMcp18_319;
    sens->A[1] = ACcp18_119;
    sens->A[2] = ACcp18_219;
    sens->A[3] = ACcp18_319;
    sens->OMP[1] = OPcp18_119;
    sens->OMP[2] = OPcp18_219;
    sens->OMP[3] = OPcp18_319;
 
// 
break;
case 20:
 


// = = Block_1_0_0_20_0_1 = = 
 
// Sensor Kinematics 


    ROcp19_45 = -S4*C5;
    ROcp19_55 = C4*C5;
    ROcp19_75 = S4*S5;
    ROcp19_85 = -C4*S5;
    ROcp19_16 = -(ROcp19_75*S6-C4*C6);
    ROcp19_26 = -(ROcp19_85*S6-S4*C6);
    ROcp19_36 = -C5*S6;
    ROcp19_76 = ROcp19_75*C6+C4*S6;
    ROcp19_86 = ROcp19_85*C6+S4*S6;
    ROcp19_96 = C5*C6;
    OMcp19_15 = qd[5]*C4;
    OMcp19_25 = qd[5]*S4;
    OMcp19_16 = OMcp19_15+ROcp19_45*qd[6];
    OMcp19_26 = OMcp19_25+ROcp19_55*qd[6];
    OMcp19_36 = qd[4]+qd[6]*S5;
    OPcp19_16 = ROcp19_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp19_25*S5-ROcp19_55*qd[4]);
    OPcp19_26 = ROcp19_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp19_15*S5-ROcp19_45*qd[4]);
    OPcp19_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_20_0_7 = = 
 
// Sensor Kinematics 


    ROcp19_420 = ROcp19_45*C20+ROcp19_76*S20;
    ROcp19_520 = ROcp19_55*C20+ROcp19_86*S20;
    ROcp19_620 = ROcp19_96*S20+C20*S5;
    ROcp19_720 = -(ROcp19_45*S20-ROcp19_76*C20);
    ROcp19_820 = -(ROcp19_55*S20-ROcp19_86*C20);
    ROcp19_920 = ROcp19_96*C20-S20*S5;
    RLcp19_120 = ROcp19_16*s->dpt[1][7]+ROcp19_45*s->dpt[2][7];
    RLcp19_220 = ROcp19_26*s->dpt[1][7]+ROcp19_55*s->dpt[2][7];
    RLcp19_320 = ROcp19_36*s->dpt[1][7]+s->dpt[2][7]*S5;
    POcp19_120 = RLcp19_120+q[1];
    POcp19_220 = RLcp19_220+q[2];
    POcp19_320 = RLcp19_320+q[3];
    OMcp19_120 = OMcp19_16+ROcp19_16*qd[20];
    OMcp19_220 = OMcp19_26+ROcp19_26*qd[20];
    OMcp19_320 = OMcp19_36+ROcp19_36*qd[20];
    ORcp19_120 = OMcp19_26*RLcp19_320-OMcp19_36*RLcp19_220;
    ORcp19_220 = -(OMcp19_16*RLcp19_320-OMcp19_36*RLcp19_120);
    ORcp19_320 = OMcp19_16*RLcp19_220-OMcp19_26*RLcp19_120;
    VIcp19_120 = ORcp19_120+qd[1];
    VIcp19_220 = ORcp19_220+qd[2];
    VIcp19_320 = ORcp19_320+qd[3];
    OPcp19_120 = OPcp19_16+ROcp19_16*qdd[20]+qd[20]*(OMcp19_26*ROcp19_36-OMcp19_36*ROcp19_26);
    OPcp19_220 = OPcp19_26+ROcp19_26*qdd[20]-qd[20]*(OMcp19_16*ROcp19_36-OMcp19_36*ROcp19_16);
    OPcp19_320 = OPcp19_36+ROcp19_36*qdd[20]+qd[20]*(OMcp19_16*ROcp19_26-OMcp19_26*ROcp19_16);
    ACcp19_120 = qdd[1]+OMcp19_26*ORcp19_320-OMcp19_36*ORcp19_220+OPcp19_26*RLcp19_320-OPcp19_36*RLcp19_220;
    ACcp19_220 = qdd[2]-OMcp19_16*ORcp19_320+OMcp19_36*ORcp19_120-OPcp19_16*RLcp19_320+OPcp19_36*RLcp19_120;
    ACcp19_320 = qdd[3]+OMcp19_16*ORcp19_220-OMcp19_26*ORcp19_120+OPcp19_16*RLcp19_220-OPcp19_26*RLcp19_120;

// = = Block_1_0_0_20_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp19_120;
    sens->P[2] = POcp19_220;
    sens->P[3] = POcp19_320;
    sens->R[1][1] = ROcp19_16;
    sens->R[1][2] = ROcp19_26;
    sens->R[1][3] = ROcp19_36;
    sens->R[2][1] = ROcp19_420;
    sens->R[2][2] = ROcp19_520;
    sens->R[2][3] = ROcp19_620;
    sens->R[3][1] = ROcp19_720;
    sens->R[3][2] = ROcp19_820;
    sens->R[3][3] = ROcp19_920;
    sens->V[1] = VIcp19_120;
    sens->V[2] = VIcp19_220;
    sens->V[3] = VIcp19_320;
    sens->OM[1] = OMcp19_120;
    sens->OM[2] = OMcp19_220;
    sens->OM[3] = OMcp19_320;
    sens->A[1] = ACcp19_120;
    sens->A[2] = ACcp19_220;
    sens->A[3] = ACcp19_320;
    sens->OMP[1] = OPcp19_120;
    sens->OMP[2] = OPcp19_220;
    sens->OMP[3] = OPcp19_320;
 
// 
break;
case 21:
 


// = = Block_1_0_0_21_0_1 = = 
 
// Sensor Kinematics 


    ROcp20_45 = -S4*C5;
    ROcp20_55 = C4*C5;
    ROcp20_75 = S4*S5;
    ROcp20_85 = -C4*S5;
    ROcp20_16 = -(ROcp20_75*S6-C4*C6);
    ROcp20_26 = -(ROcp20_85*S6-S4*C6);
    ROcp20_76 = ROcp20_75*C6+C4*S6;
    ROcp20_86 = ROcp20_85*C6+S4*S6;
    OMcp20_15 = qd[5]*C4;
    OMcp20_25 = qd[5]*S4;
    OMcp20_16 = OMcp20_15+ROcp20_45*qd[6];
    OMcp20_26 = OMcp20_25+ROcp20_55*qd[6];
    OMcp20_36 = qd[4]+qd[6]*S5;
    OPcp20_16 = ROcp20_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp20_25*S5-ROcp20_55*qd[4]);
    OPcp20_26 = ROcp20_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp20_15*S5-ROcp20_45*qd[4]);
    OPcp20_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_21_0_8 = = 
 
// Sensor Kinematics 


    ROcp20_121 = ROcp20_16*C21-ROcp20_76*S21;
    ROcp20_221 = ROcp20_26*C21-ROcp20_86*S21;
    ROcp20_321 = -S21p6*C5;
    ROcp20_721 = ROcp20_16*S21+ROcp20_76*C21;
    ROcp20_821 = ROcp20_26*S21+ROcp20_86*C21;
    ROcp20_921 = C21p6*C5;
    RLcp20_121 = ROcp20_16*s->dpt[1][8]+ROcp20_76*s->dpt[3][8];
    RLcp20_221 = ROcp20_26*s->dpt[1][8]+ROcp20_86*s->dpt[3][8];
    RLcp20_321 = -C5*(s->dpt[1][8]*S6-s->dpt[3][8]*C6);
    POcp20_121 = RLcp20_121+q[1];
    POcp20_221 = RLcp20_221+q[2];
    POcp20_321 = RLcp20_321+q[3];
    OMcp20_121 = OMcp20_16+ROcp20_45*qd[21];
    OMcp20_221 = OMcp20_26+ROcp20_55*qd[21];
    OMcp20_321 = OMcp20_36+qd[21]*S5;
    ORcp20_121 = OMcp20_26*RLcp20_321-OMcp20_36*RLcp20_221;
    ORcp20_221 = -(OMcp20_16*RLcp20_321-OMcp20_36*RLcp20_121);
    ORcp20_321 = OMcp20_16*RLcp20_221-OMcp20_26*RLcp20_121;
    VIcp20_121 = ORcp20_121+qd[1];
    VIcp20_221 = ORcp20_221+qd[2];
    VIcp20_321 = ORcp20_321+qd[3];
    OPcp20_121 = OPcp20_16+ROcp20_45*qdd[21]+qd[21]*(OMcp20_26*S5-OMcp20_36*ROcp20_55);
    OPcp20_221 = OPcp20_26+ROcp20_55*qdd[21]-qd[21]*(OMcp20_16*S5-OMcp20_36*ROcp20_45);
    OPcp20_321 = OPcp20_36+qdd[21]*S5+qd[21]*(OMcp20_16*ROcp20_55-OMcp20_26*ROcp20_45);
    ACcp20_121 = qdd[1]+OMcp20_26*ORcp20_321-OMcp20_36*ORcp20_221+OPcp20_26*RLcp20_321-OPcp20_36*RLcp20_221;
    ACcp20_221 = qdd[2]-OMcp20_16*ORcp20_321+OMcp20_36*ORcp20_121-OPcp20_16*RLcp20_321+OPcp20_36*RLcp20_121;
    ACcp20_321 = qdd[3]+OMcp20_16*ORcp20_221-OMcp20_26*ORcp20_121+OPcp20_16*RLcp20_221-OPcp20_26*RLcp20_121;

// = = Block_1_0_0_21_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp20_121;
    sens->P[2] = POcp20_221;
    sens->P[3] = POcp20_321;
    sens->R[1][1] = ROcp20_121;
    sens->R[1][2] = ROcp20_221;
    sens->R[1][3] = ROcp20_321;
    sens->R[2][1] = ROcp20_45;
    sens->R[2][2] = ROcp20_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp20_721;
    sens->R[3][2] = ROcp20_821;
    sens->R[3][3] = ROcp20_921;
    sens->V[1] = VIcp20_121;
    sens->V[2] = VIcp20_221;
    sens->V[3] = VIcp20_321;
    sens->OM[1] = OMcp20_121;
    sens->OM[2] = OMcp20_221;
    sens->OM[3] = OMcp20_321;
    sens->A[1] = ACcp20_121;
    sens->A[2] = ACcp20_221;
    sens->A[3] = ACcp20_321;
    sens->OMP[1] = OPcp20_121;
    sens->OMP[2] = OPcp20_221;
    sens->OMP[3] = OPcp20_321;
 
// 
break;
case 22:
 


// = = Block_1_0_0_22_0_1 = = 
 
// Sensor Kinematics 


    ROcp21_45 = -S4*C5;
    ROcp21_55 = C4*C5;
    ROcp21_75 = S4*S5;
    ROcp21_85 = -C4*S5;
    ROcp21_16 = -(ROcp21_75*S6-C4*C6);
    ROcp21_26 = -(ROcp21_85*S6-S4*C6);
    ROcp21_76 = ROcp21_75*C6+C4*S6;
    ROcp21_86 = ROcp21_85*C6+S4*S6;
    OMcp21_15 = qd[5]*C4;
    OMcp21_25 = qd[5]*S4;
    OMcp21_16 = OMcp21_15+ROcp21_45*qd[6];
    OMcp21_26 = OMcp21_25+ROcp21_55*qd[6];
    OMcp21_36 = qd[4]+qd[6]*S5;
    OPcp21_16 = ROcp21_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp21_25*S5-ROcp21_55*qd[4]);
    OPcp21_26 = ROcp21_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp21_15*S5-ROcp21_45*qd[4]);
    OPcp21_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_22_0_8 = = 
 
// Sensor Kinematics 


    ROcp21_121 = ROcp21_16*C21-ROcp21_76*S21;
    ROcp21_221 = ROcp21_26*C21-ROcp21_86*S21;
    ROcp21_321 = -S21p6*C5;
    ROcp21_721 = ROcp21_16*S21+ROcp21_76*C21;
    ROcp21_821 = ROcp21_26*S21+ROcp21_86*C21;
    ROcp21_921 = C21p6*C5;
    RLcp21_121 = ROcp21_16*s->dpt[1][8]+ROcp21_76*s->dpt[3][8];
    RLcp21_221 = ROcp21_26*s->dpt[1][8]+ROcp21_86*s->dpt[3][8];
    RLcp21_321 = -C5*(s->dpt[1][8]*S6-s->dpt[3][8]*C6);
    OMcp21_121 = OMcp21_16+ROcp21_45*qd[21];
    OMcp21_221 = OMcp21_26+ROcp21_55*qd[21];
    OMcp21_321 = OMcp21_36+qd[21]*S5;
    ORcp21_121 = OMcp21_26*RLcp21_321-OMcp21_36*RLcp21_221;
    ORcp21_221 = -(OMcp21_16*RLcp21_321-OMcp21_36*RLcp21_121);
    ORcp21_321 = OMcp21_16*RLcp21_221-OMcp21_26*RLcp21_121;
    OPcp21_121 = OPcp21_16+ROcp21_45*qdd[21]+qd[21]*(OMcp21_26*S5-OMcp21_36*ROcp21_55);
    OPcp21_221 = OPcp21_26+ROcp21_55*qdd[21]-qd[21]*(OMcp21_16*S5-OMcp21_36*ROcp21_45);
    OPcp21_321 = OPcp21_36+qdd[21]*S5+qd[21]*(OMcp21_16*ROcp21_55-OMcp21_26*ROcp21_45);

// = = Block_1_0_0_22_0_9 = = 
 
// Sensor Kinematics 


    ROcp21_422 = ROcp21_45*C22+ROcp21_721*S22;
    ROcp21_522 = ROcp21_55*C22+ROcp21_821*S22;
    ROcp21_622 = ROcp21_921*S22+C22*S5;
    ROcp21_722 = -(ROcp21_45*S22-ROcp21_721*C22);
    ROcp21_822 = -(ROcp21_55*S22-ROcp21_821*C22);
    ROcp21_922 = ROcp21_921*C22-S22*S5;
    RLcp21_122 = ROcp21_121*s->dpt[1][36]+ROcp21_45*s->dpt[2][36];
    RLcp21_222 = ROcp21_221*s->dpt[1][36]+ROcp21_55*s->dpt[2][36];
    RLcp21_322 = ROcp21_321*s->dpt[1][36]+s->dpt[2][36]*S5;
    POcp21_122 = RLcp21_121+RLcp21_122+q[1];
    POcp21_222 = RLcp21_221+RLcp21_222+q[2];
    POcp21_322 = RLcp21_321+RLcp21_322+q[3];
    OMcp21_122 = OMcp21_121+ROcp21_121*qd[22];
    OMcp21_222 = OMcp21_221+ROcp21_221*qd[22];
    OMcp21_322 = OMcp21_321+ROcp21_321*qd[22];
    ORcp21_122 = OMcp21_221*RLcp21_322-OMcp21_321*RLcp21_222;
    ORcp21_222 = -(OMcp21_121*RLcp21_322-OMcp21_321*RLcp21_122);
    ORcp21_322 = OMcp21_121*RLcp21_222-OMcp21_221*RLcp21_122;
    VIcp21_122 = ORcp21_121+ORcp21_122+qd[1];
    VIcp21_222 = ORcp21_221+ORcp21_222+qd[2];
    VIcp21_322 = ORcp21_321+ORcp21_322+qd[3];
    OPcp21_122 = OPcp21_121+ROcp21_121*qdd[22]+qd[22]*(OMcp21_221*ROcp21_321-OMcp21_321*ROcp21_221);
    OPcp21_222 = OPcp21_221+ROcp21_221*qdd[22]-qd[22]*(OMcp21_121*ROcp21_321-OMcp21_321*ROcp21_121);
    OPcp21_322 = OPcp21_321+ROcp21_321*qdd[22]+qd[22]*(OMcp21_121*ROcp21_221-OMcp21_221*ROcp21_121);
    ACcp21_122 = qdd[1]+OMcp21_221*ORcp21_322+OMcp21_26*ORcp21_321-OMcp21_321*ORcp21_222-OMcp21_36*ORcp21_221+OPcp21_221*
 RLcp21_322+OPcp21_26*RLcp21_321-OPcp21_321*RLcp21_222-OPcp21_36*RLcp21_221;
    ACcp21_222 = qdd[2]-OMcp21_121*ORcp21_322-OMcp21_16*ORcp21_321+OMcp21_321*ORcp21_122+OMcp21_36*ORcp21_121-OPcp21_121*
 RLcp21_322-OPcp21_16*RLcp21_321+OPcp21_321*RLcp21_122+OPcp21_36*RLcp21_121;
    ACcp21_322 = qdd[3]+OMcp21_121*ORcp21_222+OMcp21_16*ORcp21_221-OMcp21_221*ORcp21_122-OMcp21_26*ORcp21_121+OPcp21_121*
 RLcp21_222+OPcp21_16*RLcp21_221-OPcp21_221*RLcp21_122-OPcp21_26*RLcp21_121;

// = = Block_1_0_0_22_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp21_122;
    sens->P[2] = POcp21_222;
    sens->P[3] = POcp21_322;
    sens->R[1][1] = ROcp21_121;
    sens->R[1][2] = ROcp21_221;
    sens->R[1][3] = ROcp21_321;
    sens->R[2][1] = ROcp21_422;
    sens->R[2][2] = ROcp21_522;
    sens->R[2][3] = ROcp21_622;
    sens->R[3][1] = ROcp21_722;
    sens->R[3][2] = ROcp21_822;
    sens->R[3][3] = ROcp21_922;
    sens->V[1] = VIcp21_122;
    sens->V[2] = VIcp21_222;
    sens->V[3] = VIcp21_322;
    sens->OM[1] = OMcp21_122;
    sens->OM[2] = OMcp21_222;
    sens->OM[3] = OMcp21_322;
    sens->A[1] = ACcp21_122;
    sens->A[2] = ACcp21_222;
    sens->A[3] = ACcp21_322;
    sens->OMP[1] = OPcp21_122;
    sens->OMP[2] = OPcp21_222;
    sens->OMP[3] = OPcp21_322;
 
// 
break;
case 23:
 


// = = Block_1_0_0_23_0_1 = = 
 
// Sensor Kinematics 


    ROcp22_45 = -S4*C5;
    ROcp22_55 = C4*C5;
    ROcp22_75 = S4*S5;
    ROcp22_85 = -C4*S5;
    ROcp22_16 = -(ROcp22_75*S6-C4*C6);
    ROcp22_26 = -(ROcp22_85*S6-S4*C6);
    ROcp22_76 = ROcp22_75*C6+C4*S6;
    ROcp22_86 = ROcp22_85*C6+S4*S6;
    OMcp22_15 = qd[5]*C4;
    OMcp22_25 = qd[5]*S4;
    OMcp22_16 = OMcp22_15+ROcp22_45*qd[6];
    OMcp22_26 = OMcp22_25+ROcp22_55*qd[6];
    OMcp22_36 = qd[4]+qd[6]*S5;
    OPcp22_16 = ROcp22_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp22_25*S5-ROcp22_55*qd[4]);
    OPcp22_26 = ROcp22_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp22_15*S5-ROcp22_45*qd[4]);
    OPcp22_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_23_0_8 = = 
 
// Sensor Kinematics 


    ROcp22_121 = ROcp22_16*C21-ROcp22_76*S21;
    ROcp22_221 = ROcp22_26*C21-ROcp22_86*S21;
    ROcp22_321 = -S21p6*C5;
    ROcp22_721 = ROcp22_16*S21+ROcp22_76*C21;
    ROcp22_821 = ROcp22_26*S21+ROcp22_86*C21;
    ROcp22_921 = C21p6*C5;
    RLcp22_121 = ROcp22_16*s->dpt[1][8]+ROcp22_76*s->dpt[3][8];
    RLcp22_221 = ROcp22_26*s->dpt[1][8]+ROcp22_86*s->dpt[3][8];
    RLcp22_321 = -C5*(s->dpt[1][8]*S6-s->dpt[3][8]*C6);
    OMcp22_121 = OMcp22_16+ROcp22_45*qd[21];
    OMcp22_221 = OMcp22_26+ROcp22_55*qd[21];
    OMcp22_321 = OMcp22_36+qd[21]*S5;
    ORcp22_121 = OMcp22_26*RLcp22_321-OMcp22_36*RLcp22_221;
    ORcp22_221 = -(OMcp22_16*RLcp22_321-OMcp22_36*RLcp22_121);
    ORcp22_321 = OMcp22_16*RLcp22_221-OMcp22_26*RLcp22_121;
    OPcp22_121 = OPcp22_16+ROcp22_45*qdd[21]+qd[21]*(OMcp22_26*S5-OMcp22_36*ROcp22_55);
    OPcp22_221 = OPcp22_26+ROcp22_55*qdd[21]-qd[21]*(OMcp22_16*S5-OMcp22_36*ROcp22_45);
    OPcp22_321 = OPcp22_36+qdd[21]*S5+qd[21]*(OMcp22_16*ROcp22_55-OMcp22_26*ROcp22_45);

// = = Block_1_0_0_23_0_9 = = 
 
// Sensor Kinematics 


    ROcp22_422 = ROcp22_45*C22+ROcp22_721*S22;
    ROcp22_522 = ROcp22_55*C22+ROcp22_821*S22;
    ROcp22_622 = ROcp22_921*S22+C22*S5;
    ROcp22_722 = -(ROcp22_45*S22-ROcp22_721*C22);
    ROcp22_822 = -(ROcp22_55*S22-ROcp22_821*C22);
    ROcp22_922 = ROcp22_921*C22-S22*S5;
    ROcp22_123 = ROcp22_121*C23-ROcp22_722*S23;
    ROcp22_223 = ROcp22_221*C23-ROcp22_822*S23;
    ROcp22_323 = ROcp22_321*C23-ROcp22_922*S23;
    ROcp22_723 = ROcp22_121*S23+ROcp22_722*C23;
    ROcp22_823 = ROcp22_221*S23+ROcp22_822*C23;
    ROcp22_923 = ROcp22_321*S23+ROcp22_922*C23;
    RLcp22_122 = ROcp22_121*s->dpt[1][36]+ROcp22_45*s->dpt[2][36];
    RLcp22_222 = ROcp22_221*s->dpt[1][36]+ROcp22_55*s->dpt[2][36];
    RLcp22_322 = ROcp22_321*s->dpt[1][36]+s->dpt[2][36]*S5;
    POcp22_122 = RLcp22_121+RLcp22_122+q[1];
    POcp22_222 = RLcp22_221+RLcp22_222+q[2];
    POcp22_322 = RLcp22_321+RLcp22_322+q[3];
    OMcp22_122 = OMcp22_121+ROcp22_121*qd[22];
    OMcp22_222 = OMcp22_221+ROcp22_221*qd[22];
    OMcp22_322 = OMcp22_321+ROcp22_321*qd[22];
    ORcp22_122 = OMcp22_221*RLcp22_322-OMcp22_321*RLcp22_222;
    ORcp22_222 = -(OMcp22_121*RLcp22_322-OMcp22_321*RLcp22_122);
    ORcp22_322 = OMcp22_121*RLcp22_222-OMcp22_221*RLcp22_122;
    VIcp22_122 = ORcp22_121+ORcp22_122+qd[1];
    VIcp22_222 = ORcp22_221+ORcp22_222+qd[2];
    VIcp22_322 = ORcp22_321+ORcp22_322+qd[3];
    ACcp22_122 = qdd[1]+OMcp22_221*ORcp22_322+OMcp22_26*ORcp22_321-OMcp22_321*ORcp22_222-OMcp22_36*ORcp22_221+OPcp22_221*
 RLcp22_322+OPcp22_26*RLcp22_321-OPcp22_321*RLcp22_222-OPcp22_36*RLcp22_221;
    ACcp22_222 = qdd[2]-OMcp22_121*ORcp22_322-OMcp22_16*ORcp22_321+OMcp22_321*ORcp22_122+OMcp22_36*ORcp22_121-OPcp22_121*
 RLcp22_322-OPcp22_16*RLcp22_321+OPcp22_321*RLcp22_122+OPcp22_36*RLcp22_121;
    ACcp22_322 = qdd[3]+OMcp22_121*ORcp22_222+OMcp22_16*ORcp22_221-OMcp22_221*ORcp22_122-OMcp22_26*ORcp22_121+OPcp22_121*
 RLcp22_222+OPcp22_16*RLcp22_221-OPcp22_221*RLcp22_122-OPcp22_26*RLcp22_121;
    OMcp22_123 = OMcp22_122+ROcp22_422*qd[23];
    OMcp22_223 = OMcp22_222+ROcp22_522*qd[23];
    OMcp22_323 = OMcp22_322+ROcp22_622*qd[23];
    OPcp22_123 = OPcp22_121+ROcp22_121*qdd[22]+ROcp22_422*qdd[23]+qd[22]*(OMcp22_221*ROcp22_321-OMcp22_321*ROcp22_221)+
 qd[23]*(OMcp22_222*ROcp22_622-OMcp22_322*ROcp22_522);
    OPcp22_223 = OPcp22_221+ROcp22_221*qdd[22]+ROcp22_522*qdd[23]-qd[22]*(OMcp22_121*ROcp22_321-OMcp22_321*ROcp22_121)-
 qd[23]*(OMcp22_122*ROcp22_622-OMcp22_322*ROcp22_422);
    OPcp22_323 = OPcp22_321+ROcp22_321*qdd[22]+ROcp22_622*qdd[23]+qd[22]*(OMcp22_121*ROcp22_221-OMcp22_221*ROcp22_121)+
 qd[23]*(OMcp22_122*ROcp22_522-OMcp22_222*ROcp22_422);

// = = Block_1_0_0_23_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp22_122;
    sens->P[2] = POcp22_222;
    sens->P[3] = POcp22_322;
    sens->R[1][1] = ROcp22_123;
    sens->R[1][2] = ROcp22_223;
    sens->R[1][3] = ROcp22_323;
    sens->R[2][1] = ROcp22_422;
    sens->R[2][2] = ROcp22_522;
    sens->R[2][3] = ROcp22_622;
    sens->R[3][1] = ROcp22_723;
    sens->R[3][2] = ROcp22_823;
    sens->R[3][3] = ROcp22_923;
    sens->V[1] = VIcp22_122;
    sens->V[2] = VIcp22_222;
    sens->V[3] = VIcp22_322;
    sens->OM[1] = OMcp22_123;
    sens->OM[2] = OMcp22_223;
    sens->OM[3] = OMcp22_323;
    sens->A[1] = ACcp22_122;
    sens->A[2] = ACcp22_222;
    sens->A[3] = ACcp22_322;
    sens->OMP[1] = OPcp22_123;
    sens->OMP[2] = OPcp22_223;
    sens->OMP[3] = OPcp22_323;
 
// 
break;
case 24:
 


// = = Block_1_0_0_24_0_1 = = 
 
// Sensor Kinematics 


    ROcp23_45 = -S4*C5;
    ROcp23_55 = C4*C5;
    ROcp23_75 = S4*S5;
    ROcp23_85 = -C4*S5;
    ROcp23_16 = -(ROcp23_75*S6-C4*C6);
    ROcp23_26 = -(ROcp23_85*S6-S4*C6);
    ROcp23_76 = ROcp23_75*C6+C4*S6;
    ROcp23_86 = ROcp23_85*C6+S4*S6;
    OMcp23_15 = qd[5]*C4;
    OMcp23_25 = qd[5]*S4;
    OMcp23_16 = OMcp23_15+ROcp23_45*qd[6];
    OMcp23_26 = OMcp23_25+ROcp23_55*qd[6];
    OMcp23_36 = qd[4]+qd[6]*S5;
    OPcp23_16 = ROcp23_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp23_25*S5-ROcp23_55*qd[4]);
    OPcp23_26 = ROcp23_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp23_15*S5-ROcp23_45*qd[4]);
    OPcp23_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_24_0_8 = = 
 
// Sensor Kinematics 


    ROcp23_121 = ROcp23_16*C21-ROcp23_76*S21;
    ROcp23_221 = ROcp23_26*C21-ROcp23_86*S21;
    ROcp23_721 = ROcp23_16*S21+ROcp23_76*C21;
    ROcp23_821 = ROcp23_26*S21+ROcp23_86*C21;
    RLcp23_121 = ROcp23_16*s->dpt[1][8]+ROcp23_76*s->dpt[3][8];
    RLcp23_221 = ROcp23_26*s->dpt[1][8]+ROcp23_86*s->dpt[3][8];
    RLcp23_321 = -C5*(s->dpt[1][8]*S6-s->dpt[3][8]*C6);
    POcp23_121 = RLcp23_121+q[1];
    POcp23_221 = RLcp23_221+q[2];
    POcp23_321 = RLcp23_321+q[3];
    OMcp23_121 = OMcp23_16+ROcp23_45*qd[21];
    OMcp23_221 = OMcp23_26+ROcp23_55*qd[21];
    OMcp23_321 = OMcp23_36+qd[21]*S5;
    ORcp23_121 = OMcp23_26*RLcp23_321-OMcp23_36*RLcp23_221;
    ORcp23_221 = -(OMcp23_16*RLcp23_321-OMcp23_36*RLcp23_121);
    ORcp23_321 = OMcp23_16*RLcp23_221-OMcp23_26*RLcp23_121;
    VIcp23_121 = ORcp23_121+qd[1];
    VIcp23_221 = ORcp23_221+qd[2];
    VIcp23_321 = ORcp23_321+qd[3];
    ACcp23_121 = qdd[1]+OMcp23_26*ORcp23_321-OMcp23_36*ORcp23_221+OPcp23_26*RLcp23_321-OPcp23_36*RLcp23_221;
    ACcp23_221 = qdd[2]-OMcp23_16*ORcp23_321+OMcp23_36*ORcp23_121-OPcp23_16*RLcp23_321+OPcp23_36*RLcp23_121;
    ACcp23_321 = qdd[3]+OMcp23_16*ORcp23_221-OMcp23_26*ORcp23_121+OPcp23_16*RLcp23_221-OPcp23_26*RLcp23_121;

// = = Block_1_0_0_24_0_10 = = 
 
// Sensor Kinematics 


    ROcp23_124 = ROcp23_121*C24-ROcp23_721*S24;
    ROcp23_224 = ROcp23_221*C24-ROcp23_821*S24;
    ROcp23_324 = -S21p6p24*C5;
    ROcp23_724 = ROcp23_121*S24+ROcp23_721*C24;
    ROcp23_824 = ROcp23_221*S24+ROcp23_821*C24;
    ROcp23_924 = C21p6p24*C5;
    OMcp23_124 = OMcp23_121+ROcp23_45*qd[24];
    OMcp23_224 = OMcp23_221+ROcp23_55*qd[24];
    OMcp23_324 = OMcp23_321+qd[24]*S5;
    OPcp23_124 = OPcp23_16+ROcp23_45*qdd[21]+ROcp23_45*qdd[24]+qd[21]*(OMcp23_26*S5-OMcp23_36*ROcp23_55)+qd[24]*(
 OMcp23_221*S5-OMcp23_321*ROcp23_55);
    OPcp23_224 = OPcp23_26+ROcp23_55*qdd[21]+ROcp23_55*qdd[24]-qd[21]*(OMcp23_16*S5-OMcp23_36*ROcp23_45)-qd[24]*(
 OMcp23_121*S5-OMcp23_321*ROcp23_45);
    OPcp23_324 = OPcp23_36+qdd[21]*S5+qdd[24]*S5+qd[21]*(OMcp23_16*ROcp23_55-OMcp23_26*ROcp23_45)+qd[24]*(OMcp23_121*
 ROcp23_55-OMcp23_221*ROcp23_45);

// = = Block_1_0_0_24_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp23_121;
    sens->P[2] = POcp23_221;
    sens->P[3] = POcp23_321;
    sens->R[1][1] = ROcp23_124;
    sens->R[1][2] = ROcp23_224;
    sens->R[1][3] = ROcp23_324;
    sens->R[2][1] = ROcp23_45;
    sens->R[2][2] = ROcp23_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp23_724;
    sens->R[3][2] = ROcp23_824;
    sens->R[3][3] = ROcp23_924;
    sens->V[1] = VIcp23_121;
    sens->V[2] = VIcp23_221;
    sens->V[3] = VIcp23_321;
    sens->OM[1] = OMcp23_124;
    sens->OM[2] = OMcp23_224;
    sens->OM[3] = OMcp23_324;
    sens->A[1] = ACcp23_121;
    sens->A[2] = ACcp23_221;
    sens->A[3] = ACcp23_321;
    sens->OMP[1] = OPcp23_124;
    sens->OMP[2] = OPcp23_224;
    sens->OMP[3] = OPcp23_324;
 
// 
break;
case 25:
 


// = = Block_1_0_0_25_0_1 = = 
 
// Sensor Kinematics 


    ROcp24_45 = -S4*C5;
    ROcp24_55 = C4*C5;
    ROcp24_75 = S4*S5;
    ROcp24_85 = -C4*S5;
    ROcp24_16 = -(ROcp24_75*S6-C4*C6);
    ROcp24_26 = -(ROcp24_85*S6-S4*C6);
    ROcp24_76 = ROcp24_75*C6+C4*S6;
    ROcp24_86 = ROcp24_85*C6+S4*S6;
    OMcp24_15 = qd[5]*C4;
    OMcp24_25 = qd[5]*S4;
    OMcp24_16 = OMcp24_15+ROcp24_45*qd[6];
    OMcp24_26 = OMcp24_25+ROcp24_55*qd[6];
    OMcp24_36 = qd[4]+qd[6]*S5;
    OPcp24_16 = ROcp24_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp24_25*S5-ROcp24_55*qd[4]);
    OPcp24_26 = ROcp24_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp24_15*S5-ROcp24_45*qd[4]);
    OPcp24_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_25_0_8 = = 
 
// Sensor Kinematics 


    ROcp24_121 = ROcp24_16*C21-ROcp24_76*S21;
    ROcp24_221 = ROcp24_26*C21-ROcp24_86*S21;
    ROcp24_721 = ROcp24_16*S21+ROcp24_76*C21;
    ROcp24_821 = ROcp24_26*S21+ROcp24_86*C21;
    RLcp24_121 = ROcp24_16*s->dpt[1][8]+ROcp24_76*s->dpt[3][8];
    RLcp24_221 = ROcp24_26*s->dpt[1][8]+ROcp24_86*s->dpt[3][8];
    RLcp24_321 = -C5*(s->dpt[1][8]*S6-s->dpt[3][8]*C6);
    OMcp24_121 = OMcp24_16+ROcp24_45*qd[21];
    OMcp24_221 = OMcp24_26+ROcp24_55*qd[21];
    OMcp24_321 = OMcp24_36+qd[21]*S5;
    ORcp24_121 = OMcp24_26*RLcp24_321-OMcp24_36*RLcp24_221;
    ORcp24_221 = -(OMcp24_16*RLcp24_321-OMcp24_36*RLcp24_121);
    ORcp24_321 = OMcp24_16*RLcp24_221-OMcp24_26*RLcp24_121;

// = = Block_1_0_0_25_0_10 = = 
 
// Sensor Kinematics 


    ROcp24_124 = ROcp24_121*C24-ROcp24_721*S24;
    ROcp24_224 = ROcp24_221*C24-ROcp24_821*S24;
    ROcp24_324 = -S21p6p24*C5;
    ROcp24_724 = ROcp24_121*S24+ROcp24_721*C24;
    ROcp24_824 = ROcp24_221*S24+ROcp24_821*C24;
    ROcp24_924 = C21p6p24*C5;
    ROcp24_425 = ROcp24_45*C25+ROcp24_724*S25;
    ROcp24_525 = ROcp24_55*C25+ROcp24_824*S25;
    ROcp24_625 = ROcp24_924*S25+C25*S5;
    ROcp24_725 = -(ROcp24_45*S25-ROcp24_724*C25);
    ROcp24_825 = -(ROcp24_55*S25-ROcp24_824*C25);
    ROcp24_925 = ROcp24_924*C25-S25*S5;
    OMcp24_124 = OMcp24_121+ROcp24_45*qd[24];
    OMcp24_224 = OMcp24_221+ROcp24_55*qd[24];
    OMcp24_324 = OMcp24_321+qd[24]*S5;
    OPcp24_124 = OPcp24_16+ROcp24_45*qdd[21]+ROcp24_45*qdd[24]+qd[21]*(OMcp24_26*S5-OMcp24_36*ROcp24_55)+qd[24]*(
 OMcp24_221*S5-OMcp24_321*ROcp24_55);
    OPcp24_224 = OPcp24_26+ROcp24_55*qdd[21]+ROcp24_55*qdd[24]-qd[21]*(OMcp24_16*S5-OMcp24_36*ROcp24_45)-qd[24]*(
 OMcp24_121*S5-OMcp24_321*ROcp24_45);
    OPcp24_324 = OPcp24_36+qdd[21]*S5+qdd[24]*S5+qd[21]*(OMcp24_16*ROcp24_55-OMcp24_26*ROcp24_45)+qd[24]*(OMcp24_121*
 ROcp24_55-OMcp24_221*ROcp24_45);
    RLcp24_125 = ROcp24_124*s->dpt[1][39]+ROcp24_45*s->dpt[2][39];
    RLcp24_225 = ROcp24_224*s->dpt[1][39]+ROcp24_55*s->dpt[2][39];
    RLcp24_325 = ROcp24_324*s->dpt[1][39]+s->dpt[2][39]*S5;
    POcp24_125 = RLcp24_121+RLcp24_125+q[1];
    POcp24_225 = RLcp24_221+RLcp24_225+q[2];
    POcp24_325 = RLcp24_321+RLcp24_325+q[3];
    OMcp24_125 = OMcp24_124+ROcp24_124*qd[25];
    OMcp24_225 = OMcp24_224+ROcp24_224*qd[25];
    OMcp24_325 = OMcp24_324+ROcp24_324*qd[25];
    ORcp24_125 = OMcp24_224*RLcp24_325-OMcp24_324*RLcp24_225;
    ORcp24_225 = -(OMcp24_124*RLcp24_325-OMcp24_324*RLcp24_125);
    ORcp24_325 = OMcp24_124*RLcp24_225-OMcp24_224*RLcp24_125;
    VIcp24_125 = ORcp24_121+ORcp24_125+qd[1];
    VIcp24_225 = ORcp24_221+ORcp24_225+qd[2];
    VIcp24_325 = ORcp24_321+ORcp24_325+qd[3];
    OPcp24_125 = OPcp24_124+ROcp24_124*qdd[25]+qd[25]*(OMcp24_224*ROcp24_324-OMcp24_324*ROcp24_224);
    OPcp24_225 = OPcp24_224+ROcp24_224*qdd[25]-qd[25]*(OMcp24_124*ROcp24_324-OMcp24_324*ROcp24_124);
    OPcp24_325 = OPcp24_324+ROcp24_324*qdd[25]+qd[25]*(OMcp24_124*ROcp24_224-OMcp24_224*ROcp24_124);
    ACcp24_125 = qdd[1]+OMcp24_224*ORcp24_325+OMcp24_26*ORcp24_321-OMcp24_324*ORcp24_225-OMcp24_36*ORcp24_221+OPcp24_224*
 RLcp24_325+OPcp24_26*RLcp24_321-OPcp24_324*RLcp24_225-OPcp24_36*RLcp24_221;
    ACcp24_225 = qdd[2]-OMcp24_124*ORcp24_325-OMcp24_16*ORcp24_321+OMcp24_324*ORcp24_125+OMcp24_36*ORcp24_121-OPcp24_124*
 RLcp24_325-OPcp24_16*RLcp24_321+OPcp24_324*RLcp24_125+OPcp24_36*RLcp24_121;
    ACcp24_325 = qdd[3]+OMcp24_124*ORcp24_225+OMcp24_16*ORcp24_221-OMcp24_224*ORcp24_125-OMcp24_26*ORcp24_121+OPcp24_124*
 RLcp24_225+OPcp24_16*RLcp24_221-OPcp24_224*RLcp24_125-OPcp24_26*RLcp24_121;

// = = Block_1_0_0_25_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp24_125;
    sens->P[2] = POcp24_225;
    sens->P[3] = POcp24_325;
    sens->R[1][1] = ROcp24_124;
    sens->R[1][2] = ROcp24_224;
    sens->R[1][3] = ROcp24_324;
    sens->R[2][1] = ROcp24_425;
    sens->R[2][2] = ROcp24_525;
    sens->R[2][3] = ROcp24_625;
    sens->R[3][1] = ROcp24_725;
    sens->R[3][2] = ROcp24_825;
    sens->R[3][3] = ROcp24_925;
    sens->V[1] = VIcp24_125;
    sens->V[2] = VIcp24_225;
    sens->V[3] = VIcp24_325;
    sens->OM[1] = OMcp24_125;
    sens->OM[2] = OMcp24_225;
    sens->OM[3] = OMcp24_325;
    sens->A[1] = ACcp24_125;
    sens->A[2] = ACcp24_225;
    sens->A[3] = ACcp24_325;
    sens->OMP[1] = OPcp24_125;
    sens->OMP[2] = OPcp24_225;
    sens->OMP[3] = OPcp24_325;
 
// 
break;
case 26:
 


// = = Block_1_0_0_26_0_1 = = 
 
// Sensor Kinematics 


    ROcp25_45 = -S4*C5;
    ROcp25_55 = C4*C5;
    ROcp25_75 = S4*S5;
    ROcp25_85 = -C4*S5;
    ROcp25_16 = -(ROcp25_75*S6-C4*C6);
    ROcp25_26 = -(ROcp25_85*S6-S4*C6);
    ROcp25_76 = ROcp25_75*C6+C4*S6;
    ROcp25_86 = ROcp25_85*C6+S4*S6;
    OMcp25_15 = qd[5]*C4;
    OMcp25_25 = qd[5]*S4;
    OMcp25_16 = OMcp25_15+ROcp25_45*qd[6];
    OMcp25_26 = OMcp25_25+ROcp25_55*qd[6];
    OMcp25_36 = qd[4]+qd[6]*S5;
    OPcp25_16 = ROcp25_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp25_25*S5-ROcp25_55*qd[4]);
    OPcp25_26 = ROcp25_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp25_15*S5-ROcp25_45*qd[4]);
    OPcp25_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_26_0_8 = = 
 
// Sensor Kinematics 


    ROcp25_121 = ROcp25_16*C21-ROcp25_76*S21;
    ROcp25_221 = ROcp25_26*C21-ROcp25_86*S21;
    ROcp25_721 = ROcp25_16*S21+ROcp25_76*C21;
    ROcp25_821 = ROcp25_26*S21+ROcp25_86*C21;
    RLcp25_121 = ROcp25_16*s->dpt[1][8]+ROcp25_76*s->dpt[3][8];
    RLcp25_221 = ROcp25_26*s->dpt[1][8]+ROcp25_86*s->dpt[3][8];
    RLcp25_321 = -C5*(s->dpt[1][8]*S6-s->dpt[3][8]*C6);
    OMcp25_121 = OMcp25_16+ROcp25_45*qd[21];
    OMcp25_221 = OMcp25_26+ROcp25_55*qd[21];
    OMcp25_321 = OMcp25_36+qd[21]*S5;
    ORcp25_121 = OMcp25_26*RLcp25_321-OMcp25_36*RLcp25_221;
    ORcp25_221 = -(OMcp25_16*RLcp25_321-OMcp25_36*RLcp25_121);
    ORcp25_321 = OMcp25_16*RLcp25_221-OMcp25_26*RLcp25_121;

// = = Block_1_0_0_26_0_10 = = 
 
// Sensor Kinematics 


    ROcp25_124 = ROcp25_121*C24-ROcp25_721*S24;
    ROcp25_224 = ROcp25_221*C24-ROcp25_821*S24;
    ROcp25_324 = -S21p6p24*C5;
    ROcp25_724 = ROcp25_121*S24+ROcp25_721*C24;
    ROcp25_824 = ROcp25_221*S24+ROcp25_821*C24;
    ROcp25_924 = C21p6p24*C5;
    ROcp25_425 = ROcp25_45*C25+ROcp25_724*S25;
    ROcp25_525 = ROcp25_55*C25+ROcp25_824*S25;
    ROcp25_625 = ROcp25_924*S25+C25*S5;
    ROcp25_725 = -(ROcp25_45*S25-ROcp25_724*C25);
    ROcp25_825 = -(ROcp25_55*S25-ROcp25_824*C25);
    ROcp25_925 = ROcp25_924*C25-S25*S5;
    ROcp25_126 = ROcp25_124*C26-ROcp25_725*S26;
    ROcp25_226 = ROcp25_224*C26-ROcp25_825*S26;
    ROcp25_326 = ROcp25_324*C26-ROcp25_925*S26;
    ROcp25_726 = ROcp25_124*S26+ROcp25_725*C26;
    ROcp25_826 = ROcp25_224*S26+ROcp25_825*C26;
    ROcp25_926 = ROcp25_324*S26+ROcp25_925*C26;
    OMcp25_124 = OMcp25_121+ROcp25_45*qd[24];
    OMcp25_224 = OMcp25_221+ROcp25_55*qd[24];
    OMcp25_324 = OMcp25_321+qd[24]*S5;
    OPcp25_124 = OPcp25_16+ROcp25_45*qdd[21]+ROcp25_45*qdd[24]+qd[21]*(OMcp25_26*S5-OMcp25_36*ROcp25_55)+qd[24]*(
 OMcp25_221*S5-OMcp25_321*ROcp25_55);
    OPcp25_224 = OPcp25_26+ROcp25_55*qdd[21]+ROcp25_55*qdd[24]-qd[21]*(OMcp25_16*S5-OMcp25_36*ROcp25_45)-qd[24]*(
 OMcp25_121*S5-OMcp25_321*ROcp25_45);
    OPcp25_324 = OPcp25_36+qdd[21]*S5+qdd[24]*S5+qd[21]*(OMcp25_16*ROcp25_55-OMcp25_26*ROcp25_45)+qd[24]*(OMcp25_121*
 ROcp25_55-OMcp25_221*ROcp25_45);
    RLcp25_125 = ROcp25_124*s->dpt[1][39]+ROcp25_45*s->dpt[2][39];
    RLcp25_225 = ROcp25_224*s->dpt[1][39]+ROcp25_55*s->dpt[2][39];
    RLcp25_325 = ROcp25_324*s->dpt[1][39]+s->dpt[2][39]*S5;
    POcp25_125 = RLcp25_121+RLcp25_125+q[1];
    POcp25_225 = RLcp25_221+RLcp25_225+q[2];
    POcp25_325 = RLcp25_321+RLcp25_325+q[3];
    OMcp25_125 = OMcp25_124+ROcp25_124*qd[25];
    OMcp25_225 = OMcp25_224+ROcp25_224*qd[25];
    OMcp25_325 = OMcp25_324+ROcp25_324*qd[25];
    ORcp25_125 = OMcp25_224*RLcp25_325-OMcp25_324*RLcp25_225;
    ORcp25_225 = -(OMcp25_124*RLcp25_325-OMcp25_324*RLcp25_125);
    ORcp25_325 = OMcp25_124*RLcp25_225-OMcp25_224*RLcp25_125;
    VIcp25_125 = ORcp25_121+ORcp25_125+qd[1];
    VIcp25_225 = ORcp25_221+ORcp25_225+qd[2];
    VIcp25_325 = ORcp25_321+ORcp25_325+qd[3];
    ACcp25_125 = qdd[1]+OMcp25_224*ORcp25_325+OMcp25_26*ORcp25_321-OMcp25_324*ORcp25_225-OMcp25_36*ORcp25_221+OPcp25_224*
 RLcp25_325+OPcp25_26*RLcp25_321-OPcp25_324*RLcp25_225-OPcp25_36*RLcp25_221;
    ACcp25_225 = qdd[2]-OMcp25_124*ORcp25_325-OMcp25_16*ORcp25_321+OMcp25_324*ORcp25_125+OMcp25_36*ORcp25_121-OPcp25_124*
 RLcp25_325-OPcp25_16*RLcp25_321+OPcp25_324*RLcp25_125+OPcp25_36*RLcp25_121;
    ACcp25_325 = qdd[3]+OMcp25_124*ORcp25_225+OMcp25_16*ORcp25_221-OMcp25_224*ORcp25_125-OMcp25_26*ORcp25_121+OPcp25_124*
 RLcp25_225+OPcp25_16*RLcp25_221-OPcp25_224*RLcp25_125-OPcp25_26*RLcp25_121;
    OMcp25_126 = OMcp25_125+ROcp25_425*qd[26];
    OMcp25_226 = OMcp25_225+ROcp25_525*qd[26];
    OMcp25_326 = OMcp25_325+ROcp25_625*qd[26];
    OPcp25_126 = OPcp25_124+ROcp25_124*qdd[25]+ROcp25_425*qdd[26]+qd[25]*(OMcp25_224*ROcp25_324-OMcp25_324*ROcp25_224)+
 qd[26]*(OMcp25_225*ROcp25_625-OMcp25_325*ROcp25_525);
    OPcp25_226 = OPcp25_224+ROcp25_224*qdd[25]+ROcp25_525*qdd[26]-qd[25]*(OMcp25_124*ROcp25_324-OMcp25_324*ROcp25_124)-
 qd[26]*(OMcp25_125*ROcp25_625-OMcp25_325*ROcp25_425);
    OPcp25_326 = OPcp25_324+ROcp25_324*qdd[25]+ROcp25_625*qdd[26]+qd[25]*(OMcp25_124*ROcp25_224-OMcp25_224*ROcp25_124)+
 qd[26]*(OMcp25_125*ROcp25_525-OMcp25_225*ROcp25_425);

// = = Block_1_0_0_26_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp25_125;
    sens->P[2] = POcp25_225;
    sens->P[3] = POcp25_325;
    sens->R[1][1] = ROcp25_126;
    sens->R[1][2] = ROcp25_226;
    sens->R[1][3] = ROcp25_326;
    sens->R[2][1] = ROcp25_425;
    sens->R[2][2] = ROcp25_525;
    sens->R[2][3] = ROcp25_625;
    sens->R[3][1] = ROcp25_726;
    sens->R[3][2] = ROcp25_826;
    sens->R[3][3] = ROcp25_926;
    sens->V[1] = VIcp25_125;
    sens->V[2] = VIcp25_225;
    sens->V[3] = VIcp25_325;
    sens->OM[1] = OMcp25_126;
    sens->OM[2] = OMcp25_226;
    sens->OM[3] = OMcp25_326;
    sens->A[1] = ACcp25_125;
    sens->A[2] = ACcp25_225;
    sens->A[3] = ACcp25_325;
    sens->OMP[1] = OPcp25_126;
    sens->OMP[2] = OPcp25_226;
    sens->OMP[3] = OPcp25_326;
 
// 
break;
case 27:
 


// = = Block_1_0_0_27_0_1 = = 
 
// Sensor Kinematics 


    ROcp26_45 = -S4*C5;
    ROcp26_55 = C4*C5;
    ROcp26_75 = S4*S5;
    ROcp26_85 = -C4*S5;
    ROcp26_16 = -(ROcp26_75*S6-C4*C6);
    ROcp26_26 = -(ROcp26_85*S6-S4*C6);
    ROcp26_36 = -C5*S6;
    ROcp26_76 = ROcp26_75*C6+C4*S6;
    ROcp26_86 = ROcp26_85*C6+S4*S6;
    ROcp26_96 = C5*C6;
    OMcp26_15 = qd[5]*C4;
    OMcp26_25 = qd[5]*S4;
    OMcp26_16 = OMcp26_15+ROcp26_45*qd[6];
    OMcp26_26 = OMcp26_25+ROcp26_55*qd[6];
    OMcp26_36 = qd[4]+qd[6]*S5;
    OPcp26_16 = ROcp26_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp26_25*S5-ROcp26_55*qd[4]);
    OPcp26_26 = ROcp26_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp26_15*S5-ROcp26_45*qd[4]);
    OPcp26_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_27_0_11 = = 
 
// Sensor Kinematics 


    RLcp26_127 = ROcp26_16*s->dpt[1][9]+ROcp26_45*q[27]+ROcp26_76*s->dpt[3][9];
    RLcp26_227 = ROcp26_26*s->dpt[1][9]+ROcp26_55*q[27]+ROcp26_86*s->dpt[3][9];
    RLcp26_327 = ROcp26_36*s->dpt[1][9]+ROcp26_96*s->dpt[3][9]+q[27]*S5;
    POcp26_127 = RLcp26_127+q[1];
    POcp26_227 = RLcp26_227+q[2];
    POcp26_327 = RLcp26_327+q[3];
    ORcp26_127 = OMcp26_26*RLcp26_327-OMcp26_36*RLcp26_227;
    ORcp26_227 = -(OMcp26_16*RLcp26_327-OMcp26_36*RLcp26_127);
    ORcp26_327 = OMcp26_16*RLcp26_227-OMcp26_26*RLcp26_127;
    VIcp26_127 = ORcp26_127+qd[1]+ROcp26_45*qd[27];
    VIcp26_227 = ORcp26_227+qd[2]+ROcp26_55*qd[27];
    VIcp26_327 = ORcp26_327+qd[3]+qd[27]*S5;
    ACcp26_127 = qdd[1]+OMcp26_26*ORcp26_327-OMcp26_36*ORcp26_227+OPcp26_26*RLcp26_327-OPcp26_36*RLcp26_227+ROcp26_45*
 qdd[27]+(2.0)*qd[27]*(OMcp26_26*S5-OMcp26_36*ROcp26_55);
    ACcp26_227 = qdd[2]-OMcp26_16*ORcp26_327+OMcp26_36*ORcp26_127-OPcp26_16*RLcp26_327+OPcp26_36*RLcp26_127+ROcp26_55*
 qdd[27]-(2.0)*qd[27]*(OMcp26_16*S5-OMcp26_36*ROcp26_45);
    ACcp26_327 = qdd[3]+OMcp26_16*ORcp26_227-OMcp26_26*ORcp26_127+OPcp26_16*RLcp26_227-OPcp26_26*RLcp26_127+qdd[27]*S5+(2.0)*
 qd[27]*(OMcp26_16*ROcp26_55-OMcp26_26*ROcp26_45);

// = = Block_1_0_0_27_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp26_127;
    sens->P[2] = POcp26_227;
    sens->P[3] = POcp26_327;
    sens->R[1][1] = ROcp26_16;
    sens->R[1][2] = ROcp26_26;
    sens->R[1][3] = ROcp26_36;
    sens->R[2][1] = ROcp26_45;
    sens->R[2][2] = ROcp26_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp26_76;
    sens->R[3][2] = ROcp26_86;
    sens->R[3][3] = ROcp26_96;
    sens->V[1] = VIcp26_127;
    sens->V[2] = VIcp26_227;
    sens->V[3] = VIcp26_327;
    sens->OM[1] = OMcp26_16;
    sens->OM[2] = OMcp26_26;
    sens->OM[3] = OMcp26_36;
    sens->A[1] = ACcp26_127;
    sens->A[2] = ACcp26_227;
    sens->A[3] = ACcp26_327;
    sens->OMP[1] = OPcp26_16;
    sens->OMP[2] = OPcp26_26;
    sens->OMP[3] = OPcp26_36;
 
// 
break;
case 28:
 


// = = Block_1_0_0_28_0_1 = = 
 
// Sensor Kinematics 


    ROcp27_45 = -S4*C5;
    ROcp27_55 = C4*C5;
    ROcp27_75 = S4*S5;
    ROcp27_85 = -C4*S5;
    ROcp27_16 = -(ROcp27_75*S6-C4*C6);
    ROcp27_26 = -(ROcp27_85*S6-S4*C6);
    ROcp27_36 = -C5*S6;
    ROcp27_76 = ROcp27_75*C6+C4*S6;
    ROcp27_86 = ROcp27_85*C6+S4*S6;
    ROcp27_96 = C5*C6;
    OMcp27_15 = qd[5]*C4;
    OMcp27_25 = qd[5]*S4;
    OMcp27_16 = OMcp27_15+ROcp27_45*qd[6];
    OMcp27_26 = OMcp27_25+ROcp27_55*qd[6];
    OMcp27_36 = qd[4]+qd[6]*S5;
    OPcp27_16 = ROcp27_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp27_25*S5-ROcp27_55*qd[4]);
    OPcp27_26 = ROcp27_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp27_15*S5-ROcp27_45*qd[4]);
    OPcp27_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_28_0_11 = = 
 
// Sensor Kinematics 


    RLcp27_127 = ROcp27_16*s->dpt[1][9]+ROcp27_45*q[27]+ROcp27_76*s->dpt[3][9];
    RLcp27_227 = ROcp27_26*s->dpt[1][9]+ROcp27_55*q[27]+ROcp27_86*s->dpt[3][9];
    RLcp27_327 = ROcp27_36*s->dpt[1][9]+ROcp27_96*s->dpt[3][9]+q[27]*S5;
    ORcp27_127 = OMcp27_26*RLcp27_327-OMcp27_36*RLcp27_227;
    ORcp27_227 = -(OMcp27_16*RLcp27_327-OMcp27_36*RLcp27_127);
    ORcp27_327 = OMcp27_16*RLcp27_227-OMcp27_26*RLcp27_127;

// = = Block_1_0_0_28_0_12 = = 
 
// Sensor Kinematics 


    ROcp27_428 = ROcp27_45*C28+ROcp27_76*S28;
    ROcp27_528 = ROcp27_55*C28+ROcp27_86*S28;
    ROcp27_628 = ROcp27_96*S28+C28*S5;
    ROcp27_728 = -(ROcp27_45*S28-ROcp27_76*C28);
    ROcp27_828 = -(ROcp27_55*S28-ROcp27_86*C28);
    ROcp27_928 = ROcp27_96*C28-S28*S5;
    RLcp27_128 = ROcp27_45*s->dpt[2][41];
    RLcp27_228 = ROcp27_55*s->dpt[2][41];
    RLcp27_328 = s->dpt[2][41]*S5;
    POcp27_128 = RLcp27_127+RLcp27_128+q[1];
    POcp27_228 = RLcp27_227+RLcp27_228+q[2];
    POcp27_328 = RLcp27_327+RLcp27_328+q[3];
    OMcp27_128 = OMcp27_16+ROcp27_16*qd[28];
    OMcp27_228 = OMcp27_26+ROcp27_26*qd[28];
    OMcp27_328 = OMcp27_36+ROcp27_36*qd[28];
    ORcp27_128 = OMcp27_26*RLcp27_328-OMcp27_36*RLcp27_228;
    ORcp27_228 = -(OMcp27_16*RLcp27_328-OMcp27_36*RLcp27_128);
    ORcp27_328 = OMcp27_16*RLcp27_228-OMcp27_26*RLcp27_128;
    VIcp27_128 = ORcp27_127+ORcp27_128+qd[1]+ROcp27_45*qd[27];
    VIcp27_228 = ORcp27_227+ORcp27_228+qd[2]+ROcp27_55*qd[27];
    VIcp27_328 = ORcp27_327+ORcp27_328+qd[3]+qd[27]*S5;
    OPcp27_128 = OPcp27_16+ROcp27_16*qdd[28]+qd[28]*(OMcp27_26*ROcp27_36-OMcp27_36*ROcp27_26);
    OPcp27_228 = OPcp27_26+ROcp27_26*qdd[28]-qd[28]*(OMcp27_16*ROcp27_36-OMcp27_36*ROcp27_16);
    OPcp27_328 = OPcp27_36+ROcp27_36*qdd[28]+qd[28]*(OMcp27_16*ROcp27_26-OMcp27_26*ROcp27_16);
    ACcp27_128 = qdd[1]+OMcp27_26*ORcp27_327+OMcp27_26*ORcp27_328-OMcp27_36*ORcp27_227-OMcp27_36*ORcp27_228+OPcp27_26*
 RLcp27_327+OPcp27_26*RLcp27_328-OPcp27_36*RLcp27_227-OPcp27_36*RLcp27_228+ROcp27_45*qdd[27]+(2.0)*qd[27]*(OMcp27_26*S5-OMcp27_36*
 ROcp27_55);
    ACcp27_228 = qdd[2]-OMcp27_16*ORcp27_327-OMcp27_16*ORcp27_328+OMcp27_36*ORcp27_127+OMcp27_36*ORcp27_128-OPcp27_16*
 RLcp27_327-OPcp27_16*RLcp27_328+OPcp27_36*RLcp27_127+OPcp27_36*RLcp27_128+ROcp27_55*qdd[27]-(2.0)*qd[27]*(OMcp27_16*S5-OMcp27_36*
 ROcp27_45);
    ACcp27_328 = qdd[3]+OMcp27_16*ORcp27_227+OMcp27_16*ORcp27_228-OMcp27_26*ORcp27_127-OMcp27_26*ORcp27_128+OPcp27_16*
 RLcp27_227+OPcp27_16*RLcp27_228-OPcp27_26*RLcp27_127-OPcp27_26*RLcp27_128+qdd[27]*S5+(2.0)*qd[27]*(OMcp27_16*ROcp27_55-OMcp27_26*
 ROcp27_45);

// = = Block_1_0_0_28_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp27_128;
    sens->P[2] = POcp27_228;
    sens->P[3] = POcp27_328;
    sens->R[1][1] = ROcp27_16;
    sens->R[1][2] = ROcp27_26;
    sens->R[1][3] = ROcp27_36;
    sens->R[2][1] = ROcp27_428;
    sens->R[2][2] = ROcp27_528;
    sens->R[2][3] = ROcp27_628;
    sens->R[3][1] = ROcp27_728;
    sens->R[3][2] = ROcp27_828;
    sens->R[3][3] = ROcp27_928;
    sens->V[1] = VIcp27_128;
    sens->V[2] = VIcp27_228;
    sens->V[3] = VIcp27_328;
    sens->OM[1] = OMcp27_128;
    sens->OM[2] = OMcp27_228;
    sens->OM[3] = OMcp27_328;
    sens->A[1] = ACcp27_128;
    sens->A[2] = ACcp27_228;
    sens->A[3] = ACcp27_328;
    sens->OMP[1] = OPcp27_128;
    sens->OMP[2] = OPcp27_228;
    sens->OMP[3] = OPcp27_328;
 
// 
break;
case 29:
 


// = = Block_1_0_0_29_0_1 = = 
 
// Sensor Kinematics 


    ROcp28_45 = -S4*C5;
    ROcp28_55 = C4*C5;
    ROcp28_75 = S4*S5;
    ROcp28_85 = -C4*S5;
    ROcp28_16 = -(ROcp28_75*S6-C4*C6);
    ROcp28_26 = -(ROcp28_85*S6-S4*C6);
    ROcp28_36 = -C5*S6;
    ROcp28_76 = ROcp28_75*C6+C4*S6;
    ROcp28_86 = ROcp28_85*C6+S4*S6;
    ROcp28_96 = C5*C6;
    OMcp28_15 = qd[5]*C4;
    OMcp28_25 = qd[5]*S4;
    OMcp28_16 = OMcp28_15+ROcp28_45*qd[6];
    OMcp28_26 = OMcp28_25+ROcp28_55*qd[6];
    OMcp28_36 = qd[4]+qd[6]*S5;
    OPcp28_16 = ROcp28_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp28_25*S5-ROcp28_55*qd[4]);
    OPcp28_26 = ROcp28_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp28_15*S5-ROcp28_45*qd[4]);
    OPcp28_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_29_0_11 = = 
 
// Sensor Kinematics 


    RLcp28_127 = ROcp28_16*s->dpt[1][9]+ROcp28_45*q[27]+ROcp28_76*s->dpt[3][9];
    RLcp28_227 = ROcp28_26*s->dpt[1][9]+ROcp28_55*q[27]+ROcp28_86*s->dpt[3][9];
    RLcp28_327 = ROcp28_36*s->dpt[1][9]+ROcp28_96*s->dpt[3][9]+q[27]*S5;
    ORcp28_127 = OMcp28_26*RLcp28_327-OMcp28_36*RLcp28_227;
    ORcp28_227 = -(OMcp28_16*RLcp28_327-OMcp28_36*RLcp28_127);
    ORcp28_327 = OMcp28_16*RLcp28_227-OMcp28_26*RLcp28_127;

// = = Block_1_0_0_29_0_12 = = 
 
// Sensor Kinematics 


    ROcp28_428 = ROcp28_45*C28+ROcp28_76*S28;
    ROcp28_528 = ROcp28_55*C28+ROcp28_86*S28;
    ROcp28_628 = ROcp28_96*S28+C28*S5;
    ROcp28_728 = -(ROcp28_45*S28-ROcp28_76*C28);
    ROcp28_828 = -(ROcp28_55*S28-ROcp28_86*C28);
    ROcp28_928 = ROcp28_96*C28-S28*S5;
    ROcp28_129 = ROcp28_16*C29+ROcp28_428*S29;
    ROcp28_229 = ROcp28_26*C29+ROcp28_528*S29;
    ROcp28_329 = ROcp28_36*C29+ROcp28_628*S29;
    ROcp28_429 = -(ROcp28_16*S29-ROcp28_428*C29);
    ROcp28_529 = -(ROcp28_26*S29-ROcp28_528*C29);
    ROcp28_629 = -(ROcp28_36*S29-ROcp28_628*C29);
    RLcp28_128 = ROcp28_45*s->dpt[2][41];
    RLcp28_228 = ROcp28_55*s->dpt[2][41];
    RLcp28_328 = s->dpt[2][41]*S5;
    POcp28_128 = RLcp28_127+RLcp28_128+q[1];
    POcp28_228 = RLcp28_227+RLcp28_228+q[2];
    POcp28_328 = RLcp28_327+RLcp28_328+q[3];
    OMcp28_128 = OMcp28_16+ROcp28_16*qd[28];
    OMcp28_228 = OMcp28_26+ROcp28_26*qd[28];
    OMcp28_328 = OMcp28_36+ROcp28_36*qd[28];
    ORcp28_128 = OMcp28_26*RLcp28_328-OMcp28_36*RLcp28_228;
    ORcp28_228 = -(OMcp28_16*RLcp28_328-OMcp28_36*RLcp28_128);
    ORcp28_328 = OMcp28_16*RLcp28_228-OMcp28_26*RLcp28_128;
    VIcp28_128 = ORcp28_127+ORcp28_128+qd[1]+ROcp28_45*qd[27];
    VIcp28_228 = ORcp28_227+ORcp28_228+qd[2]+ROcp28_55*qd[27];
    VIcp28_328 = ORcp28_327+ORcp28_328+qd[3]+qd[27]*S5;
    ACcp28_128 = qdd[1]+OMcp28_26*ORcp28_327+OMcp28_26*ORcp28_328-OMcp28_36*ORcp28_227-OMcp28_36*ORcp28_228+OPcp28_26*
 RLcp28_327+OPcp28_26*RLcp28_328-OPcp28_36*RLcp28_227-OPcp28_36*RLcp28_228+ROcp28_45*qdd[27]+(2.0)*qd[27]*(OMcp28_26*S5-OMcp28_36*
 ROcp28_55);
    ACcp28_228 = qdd[2]-OMcp28_16*ORcp28_327-OMcp28_16*ORcp28_328+OMcp28_36*ORcp28_127+OMcp28_36*ORcp28_128-OPcp28_16*
 RLcp28_327-OPcp28_16*RLcp28_328+OPcp28_36*RLcp28_127+OPcp28_36*RLcp28_128+ROcp28_55*qdd[27]-(2.0)*qd[27]*(OMcp28_16*S5-OMcp28_36*
 ROcp28_45);
    ACcp28_328 = qdd[3]+OMcp28_16*ORcp28_227+OMcp28_16*ORcp28_228-OMcp28_26*ORcp28_127-OMcp28_26*ORcp28_128+OPcp28_16*
 RLcp28_227+OPcp28_16*RLcp28_228-OPcp28_26*RLcp28_127-OPcp28_26*RLcp28_128+qdd[27]*S5+(2.0)*qd[27]*(OMcp28_16*ROcp28_55-OMcp28_26*
 ROcp28_45);
    OMcp28_129 = OMcp28_128+ROcp28_728*qd[29];
    OMcp28_229 = OMcp28_228+ROcp28_828*qd[29];
    OMcp28_329 = OMcp28_328+ROcp28_928*qd[29];
    OPcp28_129 = OPcp28_16+ROcp28_16*qdd[28]+ROcp28_728*qdd[29]+qd[28]*(OMcp28_26*ROcp28_36-OMcp28_36*ROcp28_26)+qd[29]*(
 OMcp28_228*ROcp28_928-OMcp28_328*ROcp28_828);
    OPcp28_229 = OPcp28_26+ROcp28_26*qdd[28]+ROcp28_828*qdd[29]-qd[28]*(OMcp28_16*ROcp28_36-OMcp28_36*ROcp28_16)-qd[29]*(
 OMcp28_128*ROcp28_928-OMcp28_328*ROcp28_728);
    OPcp28_329 = OPcp28_36+ROcp28_36*qdd[28]+ROcp28_928*qdd[29]+qd[28]*(OMcp28_16*ROcp28_26-OMcp28_26*ROcp28_16)+qd[29]*(
 OMcp28_128*ROcp28_828-OMcp28_228*ROcp28_728);

// = = Block_1_0_0_29_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp28_128;
    sens->P[2] = POcp28_228;
    sens->P[3] = POcp28_328;
    sens->R[1][1] = ROcp28_129;
    sens->R[1][2] = ROcp28_229;
    sens->R[1][3] = ROcp28_329;
    sens->R[2][1] = ROcp28_429;
    sens->R[2][2] = ROcp28_529;
    sens->R[2][3] = ROcp28_629;
    sens->R[3][1] = ROcp28_728;
    sens->R[3][2] = ROcp28_828;
    sens->R[3][3] = ROcp28_928;
    sens->V[1] = VIcp28_128;
    sens->V[2] = VIcp28_228;
    sens->V[3] = VIcp28_328;
    sens->OM[1] = OMcp28_129;
    sens->OM[2] = OMcp28_229;
    sens->OM[3] = OMcp28_329;
    sens->A[1] = ACcp28_128;
    sens->A[2] = ACcp28_228;
    sens->A[3] = ACcp28_328;
    sens->OMP[1] = OPcp28_129;
    sens->OMP[2] = OPcp28_229;
    sens->OMP[3] = OPcp28_329;
 
// 
break;
case 30:
 


// = = Block_1_0_0_30_0_1 = = 
 
// Sensor Kinematics 


    ROcp29_45 = -S4*C5;
    ROcp29_55 = C4*C5;
    ROcp29_75 = S4*S5;
    ROcp29_85 = -C4*S5;
    ROcp29_16 = -(ROcp29_75*S6-C4*C6);
    ROcp29_26 = -(ROcp29_85*S6-S4*C6);
    ROcp29_36 = -C5*S6;
    ROcp29_76 = ROcp29_75*C6+C4*S6;
    ROcp29_86 = ROcp29_85*C6+S4*S6;
    ROcp29_96 = C5*C6;
    OMcp29_15 = qd[5]*C4;
    OMcp29_25 = qd[5]*S4;
    OMcp29_16 = OMcp29_15+ROcp29_45*qd[6];
    OMcp29_26 = OMcp29_25+ROcp29_55*qd[6];
    OMcp29_36 = qd[4]+qd[6]*S5;
    OPcp29_16 = ROcp29_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp29_25*S5-ROcp29_55*qd[4]);
    OPcp29_26 = ROcp29_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp29_15*S5-ROcp29_45*qd[4]);
    OPcp29_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_30_0_11 = = 
 
// Sensor Kinematics 


    RLcp29_127 = ROcp29_16*s->dpt[1][9]+ROcp29_45*q[27]+ROcp29_76*s->dpt[3][9];
    RLcp29_227 = ROcp29_26*s->dpt[1][9]+ROcp29_55*q[27]+ROcp29_86*s->dpt[3][9];
    RLcp29_327 = ROcp29_36*s->dpt[1][9]+ROcp29_96*s->dpt[3][9]+q[27]*S5;
    ORcp29_127 = OMcp29_26*RLcp29_327-OMcp29_36*RLcp29_227;
    ORcp29_227 = -(OMcp29_16*RLcp29_327-OMcp29_36*RLcp29_127);
    ORcp29_327 = OMcp29_16*RLcp29_227-OMcp29_26*RLcp29_127;

// = = Block_1_0_0_30_0_13 = = 
 
// Sensor Kinematics 


    ROcp29_430 = ROcp29_45*C30+ROcp29_76*S30;
    ROcp29_530 = ROcp29_55*C30+ROcp29_86*S30;
    ROcp29_630 = ROcp29_96*S30+C30*S5;
    ROcp29_730 = -(ROcp29_45*S30-ROcp29_76*C30);
    ROcp29_830 = -(ROcp29_55*S30-ROcp29_86*C30);
    ROcp29_930 = ROcp29_96*C30-S30*S5;
    RLcp29_130 = ROcp29_45*s->dpt[2][42];
    RLcp29_230 = ROcp29_55*s->dpt[2][42];
    RLcp29_330 = s->dpt[2][42]*S5;
    POcp29_130 = RLcp29_127+RLcp29_130+q[1];
    POcp29_230 = RLcp29_227+RLcp29_230+q[2];
    POcp29_330 = RLcp29_327+RLcp29_330+q[3];
    OMcp29_130 = OMcp29_16+ROcp29_16*qd[30];
    OMcp29_230 = OMcp29_26+ROcp29_26*qd[30];
    OMcp29_330 = OMcp29_36+ROcp29_36*qd[30];
    ORcp29_130 = OMcp29_26*RLcp29_330-OMcp29_36*RLcp29_230;
    ORcp29_230 = -(OMcp29_16*RLcp29_330-OMcp29_36*RLcp29_130);
    ORcp29_330 = OMcp29_16*RLcp29_230-OMcp29_26*RLcp29_130;
    VIcp29_130 = ORcp29_127+ORcp29_130+qd[1]+ROcp29_45*qd[27];
    VIcp29_230 = ORcp29_227+ORcp29_230+qd[2]+ROcp29_55*qd[27];
    VIcp29_330 = ORcp29_327+ORcp29_330+qd[3]+qd[27]*S5;
    OPcp29_130 = OPcp29_16+ROcp29_16*qdd[30]+qd[30]*(OMcp29_26*ROcp29_36-OMcp29_36*ROcp29_26);
    OPcp29_230 = OPcp29_26+ROcp29_26*qdd[30]-qd[30]*(OMcp29_16*ROcp29_36-OMcp29_36*ROcp29_16);
    OPcp29_330 = OPcp29_36+ROcp29_36*qdd[30]+qd[30]*(OMcp29_16*ROcp29_26-OMcp29_26*ROcp29_16);
    ACcp29_130 = qdd[1]+OMcp29_26*ORcp29_327+OMcp29_26*ORcp29_330-OMcp29_36*ORcp29_227-OMcp29_36*ORcp29_230+OPcp29_26*
 RLcp29_327+OPcp29_26*RLcp29_330-OPcp29_36*RLcp29_227-OPcp29_36*RLcp29_230+ROcp29_45*qdd[27]+(2.0)*qd[27]*(OMcp29_26*S5-OMcp29_36*
 ROcp29_55);
    ACcp29_230 = qdd[2]-OMcp29_16*ORcp29_327-OMcp29_16*ORcp29_330+OMcp29_36*ORcp29_127+OMcp29_36*ORcp29_130-OPcp29_16*
 RLcp29_327-OPcp29_16*RLcp29_330+OPcp29_36*RLcp29_127+OPcp29_36*RLcp29_130+ROcp29_55*qdd[27]-(2.0)*qd[27]*(OMcp29_16*S5-OMcp29_36*
 ROcp29_45);
    ACcp29_330 = qdd[3]+OMcp29_16*ORcp29_227+OMcp29_16*ORcp29_230-OMcp29_26*ORcp29_127-OMcp29_26*ORcp29_130+OPcp29_16*
 RLcp29_227+OPcp29_16*RLcp29_230-OPcp29_26*RLcp29_127-OPcp29_26*RLcp29_130+qdd[27]*S5+(2.0)*qd[27]*(OMcp29_16*ROcp29_55-OMcp29_26*
 ROcp29_45);

// = = Block_1_0_0_30_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp29_130;
    sens->P[2] = POcp29_230;
    sens->P[3] = POcp29_330;
    sens->R[1][1] = ROcp29_16;
    sens->R[1][2] = ROcp29_26;
    sens->R[1][3] = ROcp29_36;
    sens->R[2][1] = ROcp29_430;
    sens->R[2][2] = ROcp29_530;
    sens->R[2][3] = ROcp29_630;
    sens->R[3][1] = ROcp29_730;
    sens->R[3][2] = ROcp29_830;
    sens->R[3][3] = ROcp29_930;
    sens->V[1] = VIcp29_130;
    sens->V[2] = VIcp29_230;
    sens->V[3] = VIcp29_330;
    sens->OM[1] = OMcp29_130;
    sens->OM[2] = OMcp29_230;
    sens->OM[3] = OMcp29_330;
    sens->A[1] = ACcp29_130;
    sens->A[2] = ACcp29_230;
    sens->A[3] = ACcp29_330;
    sens->OMP[1] = OPcp29_130;
    sens->OMP[2] = OPcp29_230;
    sens->OMP[3] = OPcp29_330;
 
// 
break;
case 31:
 


// = = Block_1_0_0_31_0_1 = = 
 
// Sensor Kinematics 


    ROcp30_45 = -S4*C5;
    ROcp30_55 = C4*C5;
    ROcp30_75 = S4*S5;
    ROcp30_85 = -C4*S5;
    ROcp30_16 = -(ROcp30_75*S6-C4*C6);
    ROcp30_26 = -(ROcp30_85*S6-S4*C6);
    ROcp30_36 = -C5*S6;
    ROcp30_76 = ROcp30_75*C6+C4*S6;
    ROcp30_86 = ROcp30_85*C6+S4*S6;
    ROcp30_96 = C5*C6;
    OMcp30_15 = qd[5]*C4;
    OMcp30_25 = qd[5]*S4;
    OMcp30_16 = OMcp30_15+ROcp30_45*qd[6];
    OMcp30_26 = OMcp30_25+ROcp30_55*qd[6];
    OMcp30_36 = qd[4]+qd[6]*S5;
    OPcp30_16 = ROcp30_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp30_25*S5-ROcp30_55*qd[4]);
    OPcp30_26 = ROcp30_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp30_15*S5-ROcp30_45*qd[4]);
    OPcp30_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_31_0_11 = = 
 
// Sensor Kinematics 


    RLcp30_127 = ROcp30_16*s->dpt[1][9]+ROcp30_45*q[27]+ROcp30_76*s->dpt[3][9];
    RLcp30_227 = ROcp30_26*s->dpt[1][9]+ROcp30_55*q[27]+ROcp30_86*s->dpt[3][9];
    RLcp30_327 = ROcp30_36*s->dpt[1][9]+ROcp30_96*s->dpt[3][9]+q[27]*S5;
    ORcp30_127 = OMcp30_26*RLcp30_327-OMcp30_36*RLcp30_227;
    ORcp30_227 = -(OMcp30_16*RLcp30_327-OMcp30_36*RLcp30_127);
    ORcp30_327 = OMcp30_16*RLcp30_227-OMcp30_26*RLcp30_127;

// = = Block_1_0_0_31_0_13 = = 
 
// Sensor Kinematics 


    ROcp30_430 = ROcp30_45*C30+ROcp30_76*S30;
    ROcp30_530 = ROcp30_55*C30+ROcp30_86*S30;
    ROcp30_630 = ROcp30_96*S30+C30*S5;
    ROcp30_730 = -(ROcp30_45*S30-ROcp30_76*C30);
    ROcp30_830 = -(ROcp30_55*S30-ROcp30_86*C30);
    ROcp30_930 = ROcp30_96*C30-S30*S5;
    ROcp30_131 = ROcp30_16*C31+ROcp30_430*S31;
    ROcp30_231 = ROcp30_26*C31+ROcp30_530*S31;
    ROcp30_331 = ROcp30_36*C31+ROcp30_630*S31;
    ROcp30_431 = -(ROcp30_16*S31-ROcp30_430*C31);
    ROcp30_531 = -(ROcp30_26*S31-ROcp30_530*C31);
    ROcp30_631 = -(ROcp30_36*S31-ROcp30_630*C31);
    RLcp30_130 = ROcp30_45*s->dpt[2][42];
    RLcp30_230 = ROcp30_55*s->dpt[2][42];
    RLcp30_330 = s->dpt[2][42]*S5;
    POcp30_130 = RLcp30_127+RLcp30_130+q[1];
    POcp30_230 = RLcp30_227+RLcp30_230+q[2];
    POcp30_330 = RLcp30_327+RLcp30_330+q[3];
    OMcp30_130 = OMcp30_16+ROcp30_16*qd[30];
    OMcp30_230 = OMcp30_26+ROcp30_26*qd[30];
    OMcp30_330 = OMcp30_36+ROcp30_36*qd[30];
    ORcp30_130 = OMcp30_26*RLcp30_330-OMcp30_36*RLcp30_230;
    ORcp30_230 = -(OMcp30_16*RLcp30_330-OMcp30_36*RLcp30_130);
    ORcp30_330 = OMcp30_16*RLcp30_230-OMcp30_26*RLcp30_130;
    VIcp30_130 = ORcp30_127+ORcp30_130+qd[1]+ROcp30_45*qd[27];
    VIcp30_230 = ORcp30_227+ORcp30_230+qd[2]+ROcp30_55*qd[27];
    VIcp30_330 = ORcp30_327+ORcp30_330+qd[3]+qd[27]*S5;
    ACcp30_130 = qdd[1]+OMcp30_26*ORcp30_327+OMcp30_26*ORcp30_330-OMcp30_36*ORcp30_227-OMcp30_36*ORcp30_230+OPcp30_26*
 RLcp30_327+OPcp30_26*RLcp30_330-OPcp30_36*RLcp30_227-OPcp30_36*RLcp30_230+ROcp30_45*qdd[27]+(2.0)*qd[27]*(OMcp30_26*S5-OMcp30_36*
 ROcp30_55);
    ACcp30_230 = qdd[2]-OMcp30_16*ORcp30_327-OMcp30_16*ORcp30_330+OMcp30_36*ORcp30_127+OMcp30_36*ORcp30_130-OPcp30_16*
 RLcp30_327-OPcp30_16*RLcp30_330+OPcp30_36*RLcp30_127+OPcp30_36*RLcp30_130+ROcp30_55*qdd[27]-(2.0)*qd[27]*(OMcp30_16*S5-OMcp30_36*
 ROcp30_45);
    ACcp30_330 = qdd[3]+OMcp30_16*ORcp30_227+OMcp30_16*ORcp30_230-OMcp30_26*ORcp30_127-OMcp30_26*ORcp30_130+OPcp30_16*
 RLcp30_227+OPcp30_16*RLcp30_230-OPcp30_26*RLcp30_127-OPcp30_26*RLcp30_130+qdd[27]*S5+(2.0)*qd[27]*(OMcp30_16*ROcp30_55-OMcp30_26*
 ROcp30_45);
    OMcp30_131 = OMcp30_130+ROcp30_730*qd[31];
    OMcp30_231 = OMcp30_230+ROcp30_830*qd[31];
    OMcp30_331 = OMcp30_330+ROcp30_930*qd[31];
    OPcp30_131 = OPcp30_16+ROcp30_16*qdd[30]+ROcp30_730*qdd[31]+qd[30]*(OMcp30_26*ROcp30_36-OMcp30_36*ROcp30_26)+qd[31]*(
 OMcp30_230*ROcp30_930-OMcp30_330*ROcp30_830);
    OPcp30_231 = OPcp30_26+ROcp30_26*qdd[30]+ROcp30_830*qdd[31]-qd[30]*(OMcp30_16*ROcp30_36-OMcp30_36*ROcp30_16)-qd[31]*(
 OMcp30_130*ROcp30_930-OMcp30_330*ROcp30_730);
    OPcp30_331 = OPcp30_36+ROcp30_36*qdd[30]+ROcp30_930*qdd[31]+qd[30]*(OMcp30_16*ROcp30_26-OMcp30_26*ROcp30_16)+qd[31]*(
 OMcp30_130*ROcp30_830-OMcp30_230*ROcp30_730);

// = = Block_1_0_0_31_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp30_130;
    sens->P[2] = POcp30_230;
    sens->P[3] = POcp30_330;
    sens->R[1][1] = ROcp30_131;
    sens->R[1][2] = ROcp30_231;
    sens->R[1][3] = ROcp30_331;
    sens->R[2][1] = ROcp30_431;
    sens->R[2][2] = ROcp30_531;
    sens->R[2][3] = ROcp30_631;
    sens->R[3][1] = ROcp30_730;
    sens->R[3][2] = ROcp30_830;
    sens->R[3][3] = ROcp30_930;
    sens->V[1] = VIcp30_130;
    sens->V[2] = VIcp30_230;
    sens->V[3] = VIcp30_330;
    sens->OM[1] = OMcp30_131;
    sens->OM[2] = OMcp30_231;
    sens->OM[3] = OMcp30_331;
    sens->A[1] = ACcp30_130;
    sens->A[2] = ACcp30_230;
    sens->A[3] = ACcp30_330;
    sens->OMP[1] = OPcp30_131;
    sens->OMP[2] = OPcp30_231;
    sens->OMP[3] = OPcp30_331;
 
// 
break;
case 32:
 


// = = Block_1_0_0_32_0_1 = = 
 
// Sensor Kinematics 


    ROcp31_45 = -S4*C5;
    ROcp31_55 = C4*C5;
    ROcp31_75 = S4*S5;
    ROcp31_85 = -C4*S5;
    ROcp31_16 = -(ROcp31_75*S6-C4*C6);
    ROcp31_26 = -(ROcp31_85*S6-S4*C6);
    ROcp31_36 = -C5*S6;
    ROcp31_76 = ROcp31_75*C6+C4*S6;
    ROcp31_86 = ROcp31_85*C6+S4*S6;
    ROcp31_96 = C5*C6;
    OMcp31_15 = qd[5]*C4;
    OMcp31_25 = qd[5]*S4;
    OMcp31_16 = OMcp31_15+ROcp31_45*qd[6];
    OMcp31_26 = OMcp31_25+ROcp31_55*qd[6];
    OMcp31_36 = qd[4]+qd[6]*S5;
    OPcp31_16 = ROcp31_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp31_25*S5-ROcp31_55*qd[4]);
    OPcp31_26 = ROcp31_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp31_15*S5-ROcp31_45*qd[4]);
    OPcp31_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_32_0_14 = = 
 
// Sensor Kinematics 


    ROcp31_432 = ROcp31_45*C32+ROcp31_76*S32;
    ROcp31_532 = ROcp31_55*C32+ROcp31_86*S32;
    ROcp31_632 = ROcp31_96*S32+C32*S5;
    ROcp31_732 = -(ROcp31_45*S32-ROcp31_76*C32);
    ROcp31_832 = -(ROcp31_55*S32-ROcp31_86*C32);
    ROcp31_932 = ROcp31_96*C32-S32*S5;
    RLcp31_132 = ROcp31_16*s->dpt[1][12]+ROcp31_45*s->dpt[2][12]+ROcp31_76*s->dpt[3][12];
    RLcp31_232 = ROcp31_26*s->dpt[1][12]+ROcp31_55*s->dpt[2][12]+ROcp31_86*s->dpt[3][12];
    RLcp31_332 = ROcp31_36*s->dpt[1][12]+ROcp31_96*s->dpt[3][12]+s->dpt[2][12]*S5;
    POcp31_132 = RLcp31_132+q[1];
    POcp31_232 = RLcp31_232+q[2];
    POcp31_332 = RLcp31_332+q[3];
    OMcp31_132 = OMcp31_16+ROcp31_16*qd[32];
    OMcp31_232 = OMcp31_26+ROcp31_26*qd[32];
    OMcp31_332 = OMcp31_36+ROcp31_36*qd[32];
    ORcp31_132 = OMcp31_26*RLcp31_332-OMcp31_36*RLcp31_232;
    ORcp31_232 = -(OMcp31_16*RLcp31_332-OMcp31_36*RLcp31_132);
    ORcp31_332 = OMcp31_16*RLcp31_232-OMcp31_26*RLcp31_132;
    VIcp31_132 = ORcp31_132+qd[1];
    VIcp31_232 = ORcp31_232+qd[2];
    VIcp31_332 = ORcp31_332+qd[3];
    OPcp31_132 = OPcp31_16+ROcp31_16*qdd[32]+qd[32]*(OMcp31_26*ROcp31_36-OMcp31_36*ROcp31_26);
    OPcp31_232 = OPcp31_26+ROcp31_26*qdd[32]-qd[32]*(OMcp31_16*ROcp31_36-OMcp31_36*ROcp31_16);
    OPcp31_332 = OPcp31_36+ROcp31_36*qdd[32]+qd[32]*(OMcp31_16*ROcp31_26-OMcp31_26*ROcp31_16);
    ACcp31_132 = qdd[1]+OMcp31_26*ORcp31_332-OMcp31_36*ORcp31_232+OPcp31_26*RLcp31_332-OPcp31_36*RLcp31_232;
    ACcp31_232 = qdd[2]-OMcp31_16*ORcp31_332+OMcp31_36*ORcp31_132-OPcp31_16*RLcp31_332+OPcp31_36*RLcp31_132;
    ACcp31_332 = qdd[3]+OMcp31_16*ORcp31_232-OMcp31_26*ORcp31_132+OPcp31_16*RLcp31_232-OPcp31_26*RLcp31_132;

// = = Block_1_0_0_32_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp31_132;
    sens->P[2] = POcp31_232;
    sens->P[3] = POcp31_332;
    sens->R[1][1] = ROcp31_16;
    sens->R[1][2] = ROcp31_26;
    sens->R[1][3] = ROcp31_36;
    sens->R[2][1] = ROcp31_432;
    sens->R[2][2] = ROcp31_532;
    sens->R[2][3] = ROcp31_632;
    sens->R[3][1] = ROcp31_732;
    sens->R[3][2] = ROcp31_832;
    sens->R[3][3] = ROcp31_932;
    sens->V[1] = VIcp31_132;
    sens->V[2] = VIcp31_232;
    sens->V[3] = VIcp31_332;
    sens->OM[1] = OMcp31_132;
    sens->OM[2] = OMcp31_232;
    sens->OM[3] = OMcp31_332;
    sens->A[1] = ACcp31_132;
    sens->A[2] = ACcp31_232;
    sens->A[3] = ACcp31_332;
    sens->OMP[1] = OPcp31_132;
    sens->OMP[2] = OPcp31_232;
    sens->OMP[3] = OPcp31_332;
 
// 
break;
case 33:
 


// = = Block_1_0_0_33_0_1 = = 
 
// Sensor Kinematics 


    ROcp32_45 = -S4*C5;
    ROcp32_55 = C4*C5;
    ROcp32_75 = S4*S5;
    ROcp32_85 = -C4*S5;
    ROcp32_16 = -(ROcp32_75*S6-C4*C6);
    ROcp32_26 = -(ROcp32_85*S6-S4*C6);
    ROcp32_36 = -C5*S6;
    ROcp32_76 = ROcp32_75*C6+C4*S6;
    ROcp32_86 = ROcp32_85*C6+S4*S6;
    ROcp32_96 = C5*C6;
    OMcp32_15 = qd[5]*C4;
    OMcp32_25 = qd[5]*S4;
    OMcp32_16 = OMcp32_15+ROcp32_45*qd[6];
    OMcp32_26 = OMcp32_25+ROcp32_55*qd[6];
    OMcp32_36 = qd[4]+qd[6]*S5;
    OPcp32_16 = ROcp32_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp32_25*S5-ROcp32_55*qd[4]);
    OPcp32_26 = ROcp32_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp32_15*S5-ROcp32_45*qd[4]);
    OPcp32_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_33_0_14 = = 
 
// Sensor Kinematics 


    ROcp32_432 = ROcp32_45*C32+ROcp32_76*S32;
    ROcp32_532 = ROcp32_55*C32+ROcp32_86*S32;
    ROcp32_632 = ROcp32_96*S32+C32*S5;
    ROcp32_732 = -(ROcp32_45*S32-ROcp32_76*C32);
    ROcp32_832 = -(ROcp32_55*S32-ROcp32_86*C32);
    ROcp32_932 = ROcp32_96*C32-S32*S5;
    ROcp32_433 = ROcp32_432*C33+ROcp32_732*S33;
    ROcp32_533 = ROcp32_532*C33+ROcp32_832*S33;
    ROcp32_633 = ROcp32_632*C33+ROcp32_932*S33;
    ROcp32_733 = -(ROcp32_432*S33-ROcp32_732*C33);
    ROcp32_833 = -(ROcp32_532*S33-ROcp32_832*C33);
    ROcp32_933 = -(ROcp32_632*S33-ROcp32_932*C33);
    RLcp32_132 = ROcp32_16*s->dpt[1][12]+ROcp32_45*s->dpt[2][12]+ROcp32_76*s->dpt[3][12];
    RLcp32_232 = ROcp32_26*s->dpt[1][12]+ROcp32_55*s->dpt[2][12]+ROcp32_86*s->dpt[3][12];
    RLcp32_332 = ROcp32_36*s->dpt[1][12]+ROcp32_96*s->dpt[3][12]+s->dpt[2][12]*S5;
    OMcp32_132 = OMcp32_16+ROcp32_16*qd[32];
    OMcp32_232 = OMcp32_26+ROcp32_26*qd[32];
    OMcp32_332 = OMcp32_36+ROcp32_36*qd[32];
    ORcp32_132 = OMcp32_26*RLcp32_332-OMcp32_36*RLcp32_232;
    ORcp32_232 = -(OMcp32_16*RLcp32_332-OMcp32_36*RLcp32_132);
    ORcp32_332 = OMcp32_16*RLcp32_232-OMcp32_26*RLcp32_132;
    OPcp32_132 = OPcp32_16+ROcp32_16*qdd[32]+qd[32]*(OMcp32_26*ROcp32_36-OMcp32_36*ROcp32_26);
    OPcp32_232 = OPcp32_26+ROcp32_26*qdd[32]-qd[32]*(OMcp32_16*ROcp32_36-OMcp32_36*ROcp32_16);
    OPcp32_332 = OPcp32_36+ROcp32_36*qdd[32]+qd[32]*(OMcp32_16*ROcp32_26-OMcp32_26*ROcp32_16);
    RLcp32_133 = ROcp32_432*s->dpt[2][45];
    RLcp32_233 = ROcp32_532*s->dpt[2][45];
    RLcp32_333 = ROcp32_632*s->dpt[2][45];
    POcp32_133 = RLcp32_132+RLcp32_133+q[1];
    POcp32_233 = RLcp32_232+RLcp32_233+q[2];
    POcp32_333 = RLcp32_332+RLcp32_333+q[3];
    OMcp32_133 = OMcp32_132+ROcp32_16*qd[33];
    OMcp32_233 = OMcp32_232+ROcp32_26*qd[33];
    OMcp32_333 = OMcp32_332+ROcp32_36*qd[33];
    ORcp32_133 = OMcp32_232*RLcp32_333-OMcp32_332*RLcp32_233;
    ORcp32_233 = -(OMcp32_132*RLcp32_333-OMcp32_332*RLcp32_133);
    ORcp32_333 = OMcp32_132*RLcp32_233-OMcp32_232*RLcp32_133;
    VIcp32_133 = ORcp32_132+ORcp32_133+qd[1];
    VIcp32_233 = ORcp32_232+ORcp32_233+qd[2];
    VIcp32_333 = ORcp32_332+ORcp32_333+qd[3];
    OPcp32_133 = OPcp32_132+ROcp32_16*qdd[33]+qd[33]*(OMcp32_232*ROcp32_36-OMcp32_332*ROcp32_26);
    OPcp32_233 = OPcp32_232+ROcp32_26*qdd[33]-qd[33]*(OMcp32_132*ROcp32_36-OMcp32_332*ROcp32_16);
    OPcp32_333 = OPcp32_332+ROcp32_36*qdd[33]+qd[33]*(OMcp32_132*ROcp32_26-OMcp32_232*ROcp32_16);
    ACcp32_133 = qdd[1]+OMcp32_232*ORcp32_333+OMcp32_26*ORcp32_332-OMcp32_332*ORcp32_233-OMcp32_36*ORcp32_232+OPcp32_232*
 RLcp32_333+OPcp32_26*RLcp32_332-OPcp32_332*RLcp32_233-OPcp32_36*RLcp32_232;
    ACcp32_233 = qdd[2]-OMcp32_132*ORcp32_333-OMcp32_16*ORcp32_332+OMcp32_332*ORcp32_133+OMcp32_36*ORcp32_132-OPcp32_132*
 RLcp32_333-OPcp32_16*RLcp32_332+OPcp32_332*RLcp32_133+OPcp32_36*RLcp32_132;
    ACcp32_333 = qdd[3]+OMcp32_132*ORcp32_233+OMcp32_16*ORcp32_232-OMcp32_232*ORcp32_133-OMcp32_26*ORcp32_132+OPcp32_132*
 RLcp32_233+OPcp32_16*RLcp32_232-OPcp32_232*RLcp32_133-OPcp32_26*RLcp32_132;

// = = Block_1_0_0_33_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp32_133;
    sens->P[2] = POcp32_233;
    sens->P[3] = POcp32_333;
    sens->R[1][1] = ROcp32_16;
    sens->R[1][2] = ROcp32_26;
    sens->R[1][3] = ROcp32_36;
    sens->R[2][1] = ROcp32_433;
    sens->R[2][2] = ROcp32_533;
    sens->R[2][3] = ROcp32_633;
    sens->R[3][1] = ROcp32_733;
    sens->R[3][2] = ROcp32_833;
    sens->R[3][3] = ROcp32_933;
    sens->V[1] = VIcp32_133;
    sens->V[2] = VIcp32_233;
    sens->V[3] = VIcp32_333;
    sens->OM[1] = OMcp32_133;
    sens->OM[2] = OMcp32_233;
    sens->OM[3] = OMcp32_333;
    sens->A[1] = ACcp32_133;
    sens->A[2] = ACcp32_233;
    sens->A[3] = ACcp32_333;
    sens->OMP[1] = OPcp32_133;
    sens->OMP[2] = OPcp32_233;
    sens->OMP[3] = OPcp32_333;
 
// 
break;
case 34:
 


// = = Block_1_0_0_34_0_1 = = 
 
// Sensor Kinematics 


    ROcp33_45 = -S4*C5;
    ROcp33_55 = C4*C5;
    ROcp33_75 = S4*S5;
    ROcp33_85 = -C4*S5;
    ROcp33_16 = -(ROcp33_75*S6-C4*C6);
    ROcp33_26 = -(ROcp33_85*S6-S4*C6);
    ROcp33_36 = -C5*S6;
    ROcp33_76 = ROcp33_75*C6+C4*S6;
    ROcp33_86 = ROcp33_85*C6+S4*S6;
    ROcp33_96 = C5*C6;
    OMcp33_15 = qd[5]*C4;
    OMcp33_25 = qd[5]*S4;
    OMcp33_16 = OMcp33_15+ROcp33_45*qd[6];
    OMcp33_26 = OMcp33_25+ROcp33_55*qd[6];
    OMcp33_36 = qd[4]+qd[6]*S5;
    OPcp33_16 = ROcp33_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp33_25*S5-ROcp33_55*qd[4]);
    OPcp33_26 = ROcp33_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp33_15*S5-ROcp33_45*qd[4]);
    OPcp33_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_34_0_14 = = 
 
// Sensor Kinematics 


    ROcp33_432 = ROcp33_45*C32+ROcp33_76*S32;
    ROcp33_532 = ROcp33_55*C32+ROcp33_86*S32;
    ROcp33_632 = ROcp33_96*S32+C32*S5;
    ROcp33_732 = -(ROcp33_45*S32-ROcp33_76*C32);
    ROcp33_832 = -(ROcp33_55*S32-ROcp33_86*C32);
    ROcp33_932 = ROcp33_96*C32-S32*S5;
    ROcp33_433 = ROcp33_432*C33+ROcp33_732*S33;
    ROcp33_533 = ROcp33_532*C33+ROcp33_832*S33;
    ROcp33_633 = ROcp33_632*C33+ROcp33_932*S33;
    ROcp33_733 = -(ROcp33_432*S33-ROcp33_732*C33);
    ROcp33_833 = -(ROcp33_532*S33-ROcp33_832*C33);
    ROcp33_933 = -(ROcp33_632*S33-ROcp33_932*C33);
    ROcp33_134 = ROcp33_16*C34-ROcp33_733*S34;
    ROcp33_234 = ROcp33_26*C34-ROcp33_833*S34;
    ROcp33_334 = ROcp33_36*C34-ROcp33_933*S34;
    ROcp33_734 = ROcp33_16*S34+ROcp33_733*C34;
    ROcp33_834 = ROcp33_26*S34+ROcp33_833*C34;
    ROcp33_934 = ROcp33_36*S34+ROcp33_933*C34;
    RLcp33_132 = ROcp33_16*s->dpt[1][12]+ROcp33_45*s->dpt[2][12]+ROcp33_76*s->dpt[3][12];
    RLcp33_232 = ROcp33_26*s->dpt[1][12]+ROcp33_55*s->dpt[2][12]+ROcp33_86*s->dpt[3][12];
    RLcp33_332 = ROcp33_36*s->dpt[1][12]+ROcp33_96*s->dpt[3][12]+s->dpt[2][12]*S5;
    OMcp33_132 = OMcp33_16+ROcp33_16*qd[32];
    OMcp33_232 = OMcp33_26+ROcp33_26*qd[32];
    OMcp33_332 = OMcp33_36+ROcp33_36*qd[32];
    ORcp33_132 = OMcp33_26*RLcp33_332-OMcp33_36*RLcp33_232;
    ORcp33_232 = -(OMcp33_16*RLcp33_332-OMcp33_36*RLcp33_132);
    ORcp33_332 = OMcp33_16*RLcp33_232-OMcp33_26*RLcp33_132;
    OPcp33_132 = OPcp33_16+ROcp33_16*qdd[32]+qd[32]*(OMcp33_26*ROcp33_36-OMcp33_36*ROcp33_26);
    OPcp33_232 = OPcp33_26+ROcp33_26*qdd[32]-qd[32]*(OMcp33_16*ROcp33_36-OMcp33_36*ROcp33_16);
    OPcp33_332 = OPcp33_36+ROcp33_36*qdd[32]+qd[32]*(OMcp33_16*ROcp33_26-OMcp33_26*ROcp33_16);
    RLcp33_133 = ROcp33_432*s->dpt[2][45];
    RLcp33_233 = ROcp33_532*s->dpt[2][45];
    RLcp33_333 = ROcp33_632*s->dpt[2][45];
    OMcp33_133 = OMcp33_132+ROcp33_16*qd[33];
    OMcp33_233 = OMcp33_232+ROcp33_26*qd[33];
    OMcp33_333 = OMcp33_332+ROcp33_36*qd[33];
    ORcp33_133 = OMcp33_232*RLcp33_333-OMcp33_332*RLcp33_233;
    ORcp33_233 = -(OMcp33_132*RLcp33_333-OMcp33_332*RLcp33_133);
    ORcp33_333 = OMcp33_132*RLcp33_233-OMcp33_232*RLcp33_133;
    OPcp33_133 = OPcp33_132+ROcp33_16*qdd[33]+qd[33]*(OMcp33_232*ROcp33_36-OMcp33_332*ROcp33_26);
    OPcp33_233 = OPcp33_232+ROcp33_26*qdd[33]-qd[33]*(OMcp33_132*ROcp33_36-OMcp33_332*ROcp33_16);
    OPcp33_333 = OPcp33_332+ROcp33_36*qdd[33]+qd[33]*(OMcp33_132*ROcp33_26-OMcp33_232*ROcp33_16);
    RLcp33_134 = ROcp33_733*s->dpt[3][48];
    RLcp33_234 = ROcp33_833*s->dpt[3][48];
    RLcp33_334 = ROcp33_933*s->dpt[3][48];
    POcp33_134 = RLcp33_132+RLcp33_133+RLcp33_134+q[1];
    POcp33_234 = RLcp33_232+RLcp33_233+RLcp33_234+q[2];
    POcp33_334 = RLcp33_332+RLcp33_333+RLcp33_334+q[3];
    OMcp33_134 = OMcp33_133+ROcp33_433*qd[34];
    OMcp33_234 = OMcp33_233+ROcp33_533*qd[34];
    OMcp33_334 = OMcp33_333+ROcp33_633*qd[34];
    ORcp33_134 = OMcp33_233*RLcp33_334-OMcp33_333*RLcp33_234;
    ORcp33_234 = -(OMcp33_133*RLcp33_334-OMcp33_333*RLcp33_134);
    ORcp33_334 = OMcp33_133*RLcp33_234-OMcp33_233*RLcp33_134;
    VIcp33_134 = ORcp33_132+ORcp33_133+ORcp33_134+qd[1];
    VIcp33_234 = ORcp33_232+ORcp33_233+ORcp33_234+qd[2];
    VIcp33_334 = ORcp33_332+ORcp33_333+ORcp33_334+qd[3];
    OPcp33_134 = OPcp33_133+ROcp33_433*qdd[34]+qd[34]*(OMcp33_233*ROcp33_633-OMcp33_333*ROcp33_533);
    OPcp33_234 = OPcp33_233+ROcp33_533*qdd[34]-qd[34]*(OMcp33_133*ROcp33_633-OMcp33_333*ROcp33_433);
    OPcp33_334 = OPcp33_333+ROcp33_633*qdd[34]+qd[34]*(OMcp33_133*ROcp33_533-OMcp33_233*ROcp33_433);
    ACcp33_134 = qdd[1]+OMcp33_232*ORcp33_333+OMcp33_233*ORcp33_334+OMcp33_26*ORcp33_332-OMcp33_332*ORcp33_233-OMcp33_333*
 ORcp33_234-OMcp33_36*ORcp33_232+OPcp33_232*RLcp33_333+OPcp33_233*RLcp33_334+OPcp33_26*RLcp33_332-OPcp33_332*RLcp33_233-
 OPcp33_333*RLcp33_234-OPcp33_36*RLcp33_232;
    ACcp33_234 = qdd[2]-OMcp33_132*ORcp33_333-OMcp33_133*ORcp33_334-OMcp33_16*ORcp33_332+OMcp33_332*ORcp33_133+OMcp33_333*
 ORcp33_134+OMcp33_36*ORcp33_132-OPcp33_132*RLcp33_333-OPcp33_133*RLcp33_334-OPcp33_16*RLcp33_332+OPcp33_332*RLcp33_133+
 OPcp33_333*RLcp33_134+OPcp33_36*RLcp33_132;
    ACcp33_334 = qdd[3]+OMcp33_132*ORcp33_233+OMcp33_133*ORcp33_234+OMcp33_16*ORcp33_232-OMcp33_232*ORcp33_133-OMcp33_233*
 ORcp33_134-OMcp33_26*ORcp33_132+OPcp33_132*RLcp33_233+OPcp33_133*RLcp33_234+OPcp33_16*RLcp33_232-OPcp33_232*RLcp33_133-
 OPcp33_233*RLcp33_134-OPcp33_26*RLcp33_132;

// = = Block_1_0_0_34_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp33_134;
    sens->P[2] = POcp33_234;
    sens->P[3] = POcp33_334;
    sens->R[1][1] = ROcp33_134;
    sens->R[1][2] = ROcp33_234;
    sens->R[1][3] = ROcp33_334;
    sens->R[2][1] = ROcp33_433;
    sens->R[2][2] = ROcp33_533;
    sens->R[2][3] = ROcp33_633;
    sens->R[3][1] = ROcp33_734;
    sens->R[3][2] = ROcp33_834;
    sens->R[3][3] = ROcp33_934;
    sens->V[1] = VIcp33_134;
    sens->V[2] = VIcp33_234;
    sens->V[3] = VIcp33_334;
    sens->OM[1] = OMcp33_134;
    sens->OM[2] = OMcp33_234;
    sens->OM[3] = OMcp33_334;
    sens->A[1] = ACcp33_134;
    sens->A[2] = ACcp33_234;
    sens->A[3] = ACcp33_334;
    sens->OMP[1] = OPcp33_134;
    sens->OMP[2] = OPcp33_234;
    sens->OMP[3] = OPcp33_334;
 
// 
break;
case 35:
 


// = = Block_1_0_0_35_0_1 = = 
 
// Sensor Kinematics 


    ROcp34_45 = -S4*C5;
    ROcp34_55 = C4*C5;
    ROcp34_75 = S4*S5;
    ROcp34_85 = -C4*S5;
    ROcp34_16 = -(ROcp34_75*S6-C4*C6);
    ROcp34_26 = -(ROcp34_85*S6-S4*C6);
    ROcp34_36 = -C5*S6;
    ROcp34_76 = ROcp34_75*C6+C4*S6;
    ROcp34_86 = ROcp34_85*C6+S4*S6;
    ROcp34_96 = C5*C6;
    OMcp34_15 = qd[5]*C4;
    OMcp34_25 = qd[5]*S4;
    OMcp34_16 = OMcp34_15+ROcp34_45*qd[6];
    OMcp34_26 = OMcp34_25+ROcp34_55*qd[6];
    OMcp34_36 = qd[4]+qd[6]*S5;
    OPcp34_16 = ROcp34_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp34_25*S5-ROcp34_55*qd[4]);
    OPcp34_26 = ROcp34_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp34_15*S5-ROcp34_45*qd[4]);
    OPcp34_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_35_0_15 = = 
 
// Sensor Kinematics 


    ROcp34_435 = ROcp34_45*C35+ROcp34_76*S35;
    ROcp34_535 = ROcp34_55*C35+ROcp34_86*S35;
    ROcp34_635 = ROcp34_96*S35+C35*S5;
    ROcp34_735 = -(ROcp34_45*S35-ROcp34_76*C35);
    ROcp34_835 = -(ROcp34_55*S35-ROcp34_86*C35);
    ROcp34_935 = ROcp34_96*C35-S35*S5;
    RLcp34_135 = ROcp34_16*s->dpt[1][13]+ROcp34_45*s->dpt[2][13]+ROcp34_76*s->dpt[3][13];
    RLcp34_235 = ROcp34_26*s->dpt[1][13]+ROcp34_55*s->dpt[2][13]+ROcp34_86*s->dpt[3][13];
    RLcp34_335 = ROcp34_36*s->dpt[1][13]+ROcp34_96*s->dpt[3][13]+s->dpt[2][13]*S5;
    POcp34_135 = RLcp34_135+q[1];
    POcp34_235 = RLcp34_235+q[2];
    POcp34_335 = RLcp34_335+q[3];
    OMcp34_135 = OMcp34_16+ROcp34_16*qd[35];
    OMcp34_235 = OMcp34_26+ROcp34_26*qd[35];
    OMcp34_335 = OMcp34_36+ROcp34_36*qd[35];
    ORcp34_135 = OMcp34_26*RLcp34_335-OMcp34_36*RLcp34_235;
    ORcp34_235 = -(OMcp34_16*RLcp34_335-OMcp34_36*RLcp34_135);
    ORcp34_335 = OMcp34_16*RLcp34_235-OMcp34_26*RLcp34_135;
    VIcp34_135 = ORcp34_135+qd[1];
    VIcp34_235 = ORcp34_235+qd[2];
    VIcp34_335 = ORcp34_335+qd[3];
    OPcp34_135 = OPcp34_16+ROcp34_16*qdd[35]+qd[35]*(OMcp34_26*ROcp34_36-OMcp34_36*ROcp34_26);
    OPcp34_235 = OPcp34_26+ROcp34_26*qdd[35]-qd[35]*(OMcp34_16*ROcp34_36-OMcp34_36*ROcp34_16);
    OPcp34_335 = OPcp34_36+ROcp34_36*qdd[35]+qd[35]*(OMcp34_16*ROcp34_26-OMcp34_26*ROcp34_16);
    ACcp34_135 = qdd[1]+OMcp34_26*ORcp34_335-OMcp34_36*ORcp34_235+OPcp34_26*RLcp34_335-OPcp34_36*RLcp34_235;
    ACcp34_235 = qdd[2]-OMcp34_16*ORcp34_335+OMcp34_36*ORcp34_135-OPcp34_16*RLcp34_335+OPcp34_36*RLcp34_135;
    ACcp34_335 = qdd[3]+OMcp34_16*ORcp34_235-OMcp34_26*ORcp34_135+OPcp34_16*RLcp34_235-OPcp34_26*RLcp34_135;

// = = Block_1_0_0_35_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp34_135;
    sens->P[2] = POcp34_235;
    sens->P[3] = POcp34_335;
    sens->R[1][1] = ROcp34_16;
    sens->R[1][2] = ROcp34_26;
    sens->R[1][3] = ROcp34_36;
    sens->R[2][1] = ROcp34_435;
    sens->R[2][2] = ROcp34_535;
    sens->R[2][3] = ROcp34_635;
    sens->R[3][1] = ROcp34_735;
    sens->R[3][2] = ROcp34_835;
    sens->R[3][3] = ROcp34_935;
    sens->V[1] = VIcp34_135;
    sens->V[2] = VIcp34_235;
    sens->V[3] = VIcp34_335;
    sens->OM[1] = OMcp34_135;
    sens->OM[2] = OMcp34_235;
    sens->OM[3] = OMcp34_335;
    sens->A[1] = ACcp34_135;
    sens->A[2] = ACcp34_235;
    sens->A[3] = ACcp34_335;
    sens->OMP[1] = OPcp34_135;
    sens->OMP[2] = OPcp34_235;
    sens->OMP[3] = OPcp34_335;
 
// 
break;
case 36:
 


// = = Block_1_0_0_36_0_1 = = 
 
// Sensor Kinematics 


    ROcp35_45 = -S4*C5;
    ROcp35_55 = C4*C5;
    ROcp35_75 = S4*S5;
    ROcp35_85 = -C4*S5;
    ROcp35_16 = -(ROcp35_75*S6-C4*C6);
    ROcp35_26 = -(ROcp35_85*S6-S4*C6);
    ROcp35_36 = -C5*S6;
    ROcp35_76 = ROcp35_75*C6+C4*S6;
    ROcp35_86 = ROcp35_85*C6+S4*S6;
    ROcp35_96 = C5*C6;
    OMcp35_15 = qd[5]*C4;
    OMcp35_25 = qd[5]*S4;
    OMcp35_16 = OMcp35_15+ROcp35_45*qd[6];
    OMcp35_26 = OMcp35_25+ROcp35_55*qd[6];
    OMcp35_36 = qd[4]+qd[6]*S5;
    OPcp35_16 = ROcp35_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp35_25*S5-ROcp35_55*qd[4]);
    OPcp35_26 = ROcp35_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp35_15*S5-ROcp35_45*qd[4]);
    OPcp35_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_36_0_15 = = 
 
// Sensor Kinematics 


    ROcp35_435 = ROcp35_45*C35+ROcp35_76*S35;
    ROcp35_535 = ROcp35_55*C35+ROcp35_86*S35;
    ROcp35_635 = ROcp35_96*S35+C35*S5;
    ROcp35_735 = -(ROcp35_45*S35-ROcp35_76*C35);
    ROcp35_835 = -(ROcp35_55*S35-ROcp35_86*C35);
    ROcp35_935 = ROcp35_96*C35-S35*S5;
    ROcp35_436 = ROcp35_435*C36+ROcp35_735*S36;
    ROcp35_536 = ROcp35_535*C36+ROcp35_835*S36;
    ROcp35_636 = ROcp35_635*C36+ROcp35_935*S36;
    ROcp35_736 = -(ROcp35_435*S36-ROcp35_735*C36);
    ROcp35_836 = -(ROcp35_535*S36-ROcp35_835*C36);
    ROcp35_936 = -(ROcp35_635*S36-ROcp35_935*C36);
    RLcp35_135 = ROcp35_16*s->dpt[1][13]+ROcp35_45*s->dpt[2][13]+ROcp35_76*s->dpt[3][13];
    RLcp35_235 = ROcp35_26*s->dpt[1][13]+ROcp35_55*s->dpt[2][13]+ROcp35_86*s->dpt[3][13];
    RLcp35_335 = ROcp35_36*s->dpt[1][13]+ROcp35_96*s->dpt[3][13]+s->dpt[2][13]*S5;
    OMcp35_135 = OMcp35_16+ROcp35_16*qd[35];
    OMcp35_235 = OMcp35_26+ROcp35_26*qd[35];
    OMcp35_335 = OMcp35_36+ROcp35_36*qd[35];
    ORcp35_135 = OMcp35_26*RLcp35_335-OMcp35_36*RLcp35_235;
    ORcp35_235 = -(OMcp35_16*RLcp35_335-OMcp35_36*RLcp35_135);
    ORcp35_335 = OMcp35_16*RLcp35_235-OMcp35_26*RLcp35_135;
    OPcp35_135 = OPcp35_16+ROcp35_16*qdd[35]+qd[35]*(OMcp35_26*ROcp35_36-OMcp35_36*ROcp35_26);
    OPcp35_235 = OPcp35_26+ROcp35_26*qdd[35]-qd[35]*(OMcp35_16*ROcp35_36-OMcp35_36*ROcp35_16);
    OPcp35_335 = OPcp35_36+ROcp35_36*qdd[35]+qd[35]*(OMcp35_16*ROcp35_26-OMcp35_26*ROcp35_16);
    RLcp35_136 = ROcp35_435*s->dpt[2][50];
    RLcp35_236 = ROcp35_535*s->dpt[2][50];
    RLcp35_336 = ROcp35_635*s->dpt[2][50];
    POcp35_136 = RLcp35_135+RLcp35_136+q[1];
    POcp35_236 = RLcp35_235+RLcp35_236+q[2];
    POcp35_336 = RLcp35_335+RLcp35_336+q[3];
    OMcp35_136 = OMcp35_135+ROcp35_16*qd[36];
    OMcp35_236 = OMcp35_235+ROcp35_26*qd[36];
    OMcp35_336 = OMcp35_335+ROcp35_36*qd[36];
    ORcp35_136 = OMcp35_235*RLcp35_336-OMcp35_335*RLcp35_236;
    ORcp35_236 = -(OMcp35_135*RLcp35_336-OMcp35_335*RLcp35_136);
    ORcp35_336 = OMcp35_135*RLcp35_236-OMcp35_235*RLcp35_136;
    VIcp35_136 = ORcp35_135+ORcp35_136+qd[1];
    VIcp35_236 = ORcp35_235+ORcp35_236+qd[2];
    VIcp35_336 = ORcp35_335+ORcp35_336+qd[3];
    OPcp35_136 = OPcp35_135+ROcp35_16*qdd[36]+qd[36]*(OMcp35_235*ROcp35_36-OMcp35_335*ROcp35_26);
    OPcp35_236 = OPcp35_235+ROcp35_26*qdd[36]-qd[36]*(OMcp35_135*ROcp35_36-OMcp35_335*ROcp35_16);
    OPcp35_336 = OPcp35_335+ROcp35_36*qdd[36]+qd[36]*(OMcp35_135*ROcp35_26-OMcp35_235*ROcp35_16);
    ACcp35_136 = qdd[1]+OMcp35_235*ORcp35_336+OMcp35_26*ORcp35_335-OMcp35_335*ORcp35_236-OMcp35_36*ORcp35_235+OPcp35_235*
 RLcp35_336+OPcp35_26*RLcp35_335-OPcp35_335*RLcp35_236-OPcp35_36*RLcp35_235;
    ACcp35_236 = qdd[2]-OMcp35_135*ORcp35_336-OMcp35_16*ORcp35_335+OMcp35_335*ORcp35_136+OMcp35_36*ORcp35_135-OPcp35_135*
 RLcp35_336-OPcp35_16*RLcp35_335+OPcp35_335*RLcp35_136+OPcp35_36*RLcp35_135;
    ACcp35_336 = qdd[3]+OMcp35_135*ORcp35_236+OMcp35_16*ORcp35_235-OMcp35_235*ORcp35_136-OMcp35_26*ORcp35_135+OPcp35_135*
 RLcp35_236+OPcp35_16*RLcp35_235-OPcp35_235*RLcp35_136-OPcp35_26*RLcp35_135;

// = = Block_1_0_0_36_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp35_136;
    sens->P[2] = POcp35_236;
    sens->P[3] = POcp35_336;
    sens->R[1][1] = ROcp35_16;
    sens->R[1][2] = ROcp35_26;
    sens->R[1][3] = ROcp35_36;
    sens->R[2][1] = ROcp35_436;
    sens->R[2][2] = ROcp35_536;
    sens->R[2][3] = ROcp35_636;
    sens->R[3][1] = ROcp35_736;
    sens->R[3][2] = ROcp35_836;
    sens->R[3][3] = ROcp35_936;
    sens->V[1] = VIcp35_136;
    sens->V[2] = VIcp35_236;
    sens->V[3] = VIcp35_336;
    sens->OM[1] = OMcp35_136;
    sens->OM[2] = OMcp35_236;
    sens->OM[3] = OMcp35_336;
    sens->A[1] = ACcp35_136;
    sens->A[2] = ACcp35_236;
    sens->A[3] = ACcp35_336;
    sens->OMP[1] = OPcp35_136;
    sens->OMP[2] = OPcp35_236;
    sens->OMP[3] = OPcp35_336;
 
// 
break;
case 37:
 


// = = Block_1_0_0_37_0_1 = = 
 
// Sensor Kinematics 


    ROcp36_45 = -S4*C5;
    ROcp36_55 = C4*C5;
    ROcp36_75 = S4*S5;
    ROcp36_85 = -C4*S5;
    ROcp36_16 = -(ROcp36_75*S6-C4*C6);
    ROcp36_26 = -(ROcp36_85*S6-S4*C6);
    ROcp36_36 = -C5*S6;
    ROcp36_76 = ROcp36_75*C6+C4*S6;
    ROcp36_86 = ROcp36_85*C6+S4*S6;
    ROcp36_96 = C5*C6;
    OMcp36_15 = qd[5]*C4;
    OMcp36_25 = qd[5]*S4;
    OMcp36_16 = OMcp36_15+ROcp36_45*qd[6];
    OMcp36_26 = OMcp36_25+ROcp36_55*qd[6];
    OMcp36_36 = qd[4]+qd[6]*S5;
    OPcp36_16 = ROcp36_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp36_25*S5-ROcp36_55*qd[4]);
    OPcp36_26 = ROcp36_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp36_15*S5-ROcp36_45*qd[4]);
    OPcp36_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_37_0_15 = = 
 
// Sensor Kinematics 


    ROcp36_435 = ROcp36_45*C35+ROcp36_76*S35;
    ROcp36_535 = ROcp36_55*C35+ROcp36_86*S35;
    ROcp36_635 = ROcp36_96*S35+C35*S5;
    ROcp36_735 = -(ROcp36_45*S35-ROcp36_76*C35);
    ROcp36_835 = -(ROcp36_55*S35-ROcp36_86*C35);
    ROcp36_935 = ROcp36_96*C35-S35*S5;
    ROcp36_436 = ROcp36_435*C36+ROcp36_735*S36;
    ROcp36_536 = ROcp36_535*C36+ROcp36_835*S36;
    ROcp36_636 = ROcp36_635*C36+ROcp36_935*S36;
    ROcp36_736 = -(ROcp36_435*S36-ROcp36_735*C36);
    ROcp36_836 = -(ROcp36_535*S36-ROcp36_835*C36);
    ROcp36_936 = -(ROcp36_635*S36-ROcp36_935*C36);
    ROcp36_137 = ROcp36_16*C37-ROcp36_736*S37;
    ROcp36_237 = ROcp36_26*C37-ROcp36_836*S37;
    ROcp36_337 = ROcp36_36*C37-ROcp36_936*S37;
    ROcp36_737 = ROcp36_16*S37+ROcp36_736*C37;
    ROcp36_837 = ROcp36_26*S37+ROcp36_836*C37;
    ROcp36_937 = ROcp36_36*S37+ROcp36_936*C37;
    RLcp36_135 = ROcp36_16*s->dpt[1][13]+ROcp36_45*s->dpt[2][13]+ROcp36_76*s->dpt[3][13];
    RLcp36_235 = ROcp36_26*s->dpt[1][13]+ROcp36_55*s->dpt[2][13]+ROcp36_86*s->dpt[3][13];
    RLcp36_335 = ROcp36_36*s->dpt[1][13]+ROcp36_96*s->dpt[3][13]+s->dpt[2][13]*S5;
    OMcp36_135 = OMcp36_16+ROcp36_16*qd[35];
    OMcp36_235 = OMcp36_26+ROcp36_26*qd[35];
    OMcp36_335 = OMcp36_36+ROcp36_36*qd[35];
    ORcp36_135 = OMcp36_26*RLcp36_335-OMcp36_36*RLcp36_235;
    ORcp36_235 = -(OMcp36_16*RLcp36_335-OMcp36_36*RLcp36_135);
    ORcp36_335 = OMcp36_16*RLcp36_235-OMcp36_26*RLcp36_135;
    OPcp36_135 = OPcp36_16+ROcp36_16*qdd[35]+qd[35]*(OMcp36_26*ROcp36_36-OMcp36_36*ROcp36_26);
    OPcp36_235 = OPcp36_26+ROcp36_26*qdd[35]-qd[35]*(OMcp36_16*ROcp36_36-OMcp36_36*ROcp36_16);
    OPcp36_335 = OPcp36_36+ROcp36_36*qdd[35]+qd[35]*(OMcp36_16*ROcp36_26-OMcp36_26*ROcp36_16);
    RLcp36_136 = ROcp36_435*s->dpt[2][50];
    RLcp36_236 = ROcp36_535*s->dpt[2][50];
    RLcp36_336 = ROcp36_635*s->dpt[2][50];
    OMcp36_136 = OMcp36_135+ROcp36_16*qd[36];
    OMcp36_236 = OMcp36_235+ROcp36_26*qd[36];
    OMcp36_336 = OMcp36_335+ROcp36_36*qd[36];
    ORcp36_136 = OMcp36_235*RLcp36_336-OMcp36_335*RLcp36_236;
    ORcp36_236 = -(OMcp36_135*RLcp36_336-OMcp36_335*RLcp36_136);
    ORcp36_336 = OMcp36_135*RLcp36_236-OMcp36_235*RLcp36_136;
    OPcp36_136 = OPcp36_135+ROcp36_16*qdd[36]+qd[36]*(OMcp36_235*ROcp36_36-OMcp36_335*ROcp36_26);
    OPcp36_236 = OPcp36_235+ROcp36_26*qdd[36]-qd[36]*(OMcp36_135*ROcp36_36-OMcp36_335*ROcp36_16);
    OPcp36_336 = OPcp36_335+ROcp36_36*qdd[36]+qd[36]*(OMcp36_135*ROcp36_26-OMcp36_235*ROcp36_16);
    RLcp36_137 = ROcp36_736*s->dpt[3][52];
    RLcp36_237 = ROcp36_836*s->dpt[3][52];
    RLcp36_337 = ROcp36_936*s->dpt[3][52];
    POcp36_137 = RLcp36_135+RLcp36_136+RLcp36_137+q[1];
    POcp36_237 = RLcp36_235+RLcp36_236+RLcp36_237+q[2];
    POcp36_337 = RLcp36_335+RLcp36_336+RLcp36_337+q[3];
    OMcp36_137 = OMcp36_136+ROcp36_436*qd[37];
    OMcp36_237 = OMcp36_236+ROcp36_536*qd[37];
    OMcp36_337 = OMcp36_336+ROcp36_636*qd[37];
    ORcp36_137 = OMcp36_236*RLcp36_337-OMcp36_336*RLcp36_237;
    ORcp36_237 = -(OMcp36_136*RLcp36_337-OMcp36_336*RLcp36_137);
    ORcp36_337 = OMcp36_136*RLcp36_237-OMcp36_236*RLcp36_137;
    VIcp36_137 = ORcp36_135+ORcp36_136+ORcp36_137+qd[1];
    VIcp36_237 = ORcp36_235+ORcp36_236+ORcp36_237+qd[2];
    VIcp36_337 = ORcp36_335+ORcp36_336+ORcp36_337+qd[3];
    OPcp36_137 = OPcp36_136+ROcp36_436*qdd[37]+qd[37]*(OMcp36_236*ROcp36_636-OMcp36_336*ROcp36_536);
    OPcp36_237 = OPcp36_236+ROcp36_536*qdd[37]-qd[37]*(OMcp36_136*ROcp36_636-OMcp36_336*ROcp36_436);
    OPcp36_337 = OPcp36_336+ROcp36_636*qdd[37]+qd[37]*(OMcp36_136*ROcp36_536-OMcp36_236*ROcp36_436);
    ACcp36_137 = qdd[1]+OMcp36_235*ORcp36_336+OMcp36_236*ORcp36_337+OMcp36_26*ORcp36_335-OMcp36_335*ORcp36_236-OMcp36_336*
 ORcp36_237-OMcp36_36*ORcp36_235+OPcp36_235*RLcp36_336+OPcp36_236*RLcp36_337+OPcp36_26*RLcp36_335-OPcp36_335*RLcp36_236-
 OPcp36_336*RLcp36_237-OPcp36_36*RLcp36_235;
    ACcp36_237 = qdd[2]-OMcp36_135*ORcp36_336-OMcp36_136*ORcp36_337-OMcp36_16*ORcp36_335+OMcp36_335*ORcp36_136+OMcp36_336*
 ORcp36_137+OMcp36_36*ORcp36_135-OPcp36_135*RLcp36_336-OPcp36_136*RLcp36_337-OPcp36_16*RLcp36_335+OPcp36_335*RLcp36_136+
 OPcp36_336*RLcp36_137+OPcp36_36*RLcp36_135;
    ACcp36_337 = qdd[3]+OMcp36_135*ORcp36_236+OMcp36_136*ORcp36_237+OMcp36_16*ORcp36_235-OMcp36_235*ORcp36_136-OMcp36_236*
 ORcp36_137-OMcp36_26*ORcp36_135+OPcp36_135*RLcp36_236+OPcp36_136*RLcp36_237+OPcp36_16*RLcp36_235-OPcp36_235*RLcp36_136-
 OPcp36_236*RLcp36_137-OPcp36_26*RLcp36_135;

// = = Block_1_0_0_37_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp36_137;
    sens->P[2] = POcp36_237;
    sens->P[3] = POcp36_337;
    sens->R[1][1] = ROcp36_137;
    sens->R[1][2] = ROcp36_237;
    sens->R[1][3] = ROcp36_337;
    sens->R[2][1] = ROcp36_436;
    sens->R[2][2] = ROcp36_536;
    sens->R[2][3] = ROcp36_636;
    sens->R[3][1] = ROcp36_737;
    sens->R[3][2] = ROcp36_837;
    sens->R[3][3] = ROcp36_937;
    sens->V[1] = VIcp36_137;
    sens->V[2] = VIcp36_237;
    sens->V[3] = VIcp36_337;
    sens->OM[1] = OMcp36_137;
    sens->OM[2] = OMcp36_237;
    sens->OM[3] = OMcp36_337;
    sens->A[1] = ACcp36_137;
    sens->A[2] = ACcp36_237;
    sens->A[3] = ACcp36_337;
    sens->OMP[1] = OPcp36_137;
    sens->OMP[2] = OPcp36_237;
    sens->OMP[3] = OPcp36_337;
 
// 
break;
case 38:
 


// = = Block_1_0_0_38_0_1 = = 
 
// Sensor Kinematics 


    ROcp37_45 = -S4*C5;
    ROcp37_55 = C4*C5;
    ROcp37_75 = S4*S5;
    ROcp37_85 = -C4*S5;
    ROcp37_16 = -(ROcp37_75*S6-C4*C6);
    ROcp37_26 = -(ROcp37_85*S6-S4*C6);
    ROcp37_76 = ROcp37_75*C6+C4*S6;
    ROcp37_86 = ROcp37_85*C6+S4*S6;
    OMcp37_15 = qd[5]*C4;
    OMcp37_25 = qd[5]*S4;
    OMcp37_16 = OMcp37_15+ROcp37_45*qd[6];
    OMcp37_26 = OMcp37_25+ROcp37_55*qd[6];
    OMcp37_36 = qd[4]+qd[6]*S5;
    OPcp37_16 = ROcp37_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp37_25*S5-ROcp37_55*qd[4]);
    OPcp37_26 = ROcp37_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp37_15*S5-ROcp37_45*qd[4]);
    OPcp37_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_38_0_16 = = 
 
// Sensor Kinematics 


    ROcp37_138 = ROcp37_16*C38-ROcp37_76*S38;
    ROcp37_238 = ROcp37_26*C38-ROcp37_86*S38;
    ROcp37_338 = -S38p6*C5;
    ROcp37_738 = ROcp37_16*S38+ROcp37_76*C38;
    ROcp37_838 = ROcp37_26*S38+ROcp37_86*C38;
    ROcp37_938 = C38p6*C5;
    RLcp37_138 = ROcp37_16*s->dpt[1][15]+ROcp37_76*s->dpt[3][15];
    RLcp37_238 = ROcp37_26*s->dpt[1][15]+ROcp37_86*s->dpt[3][15];
    RLcp37_338 = -C5*(s->dpt[1][15]*S6-s->dpt[3][15]*C6);
    POcp37_138 = RLcp37_138+q[1];
    POcp37_238 = RLcp37_238+q[2];
    POcp37_338 = RLcp37_338+q[3];
    OMcp37_138 = OMcp37_16+ROcp37_45*qd[38];
    OMcp37_238 = OMcp37_26+ROcp37_55*qd[38];
    OMcp37_338 = OMcp37_36+qd[38]*S5;
    ORcp37_138 = OMcp37_26*RLcp37_338-OMcp37_36*RLcp37_238;
    ORcp37_238 = -(OMcp37_16*RLcp37_338-OMcp37_36*RLcp37_138);
    ORcp37_338 = OMcp37_16*RLcp37_238-OMcp37_26*RLcp37_138;
    VIcp37_138 = ORcp37_138+qd[1];
    VIcp37_238 = ORcp37_238+qd[2];
    VIcp37_338 = ORcp37_338+qd[3];
    OPcp37_138 = OPcp37_16+ROcp37_45*qdd[38]+qd[38]*(OMcp37_26*S5-OMcp37_36*ROcp37_55);
    OPcp37_238 = OPcp37_26+ROcp37_55*qdd[38]-qd[38]*(OMcp37_16*S5-OMcp37_36*ROcp37_45);
    OPcp37_338 = OPcp37_36+qdd[38]*S5+qd[38]*(OMcp37_16*ROcp37_55-OMcp37_26*ROcp37_45);
    ACcp37_138 = qdd[1]+OMcp37_26*ORcp37_338-OMcp37_36*ORcp37_238+OPcp37_26*RLcp37_338-OPcp37_36*RLcp37_238;
    ACcp37_238 = qdd[2]-OMcp37_16*ORcp37_338+OMcp37_36*ORcp37_138-OPcp37_16*RLcp37_338+OPcp37_36*RLcp37_138;
    ACcp37_338 = qdd[3]+OMcp37_16*ORcp37_238-OMcp37_26*ORcp37_138+OPcp37_16*RLcp37_238-OPcp37_26*RLcp37_138;

// = = Block_1_0_0_38_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp37_138;
    sens->P[2] = POcp37_238;
    sens->P[3] = POcp37_338;
    sens->R[1][1] = ROcp37_138;
    sens->R[1][2] = ROcp37_238;
    sens->R[1][3] = ROcp37_338;
    sens->R[2][1] = ROcp37_45;
    sens->R[2][2] = ROcp37_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp37_738;
    sens->R[3][2] = ROcp37_838;
    sens->R[3][3] = ROcp37_938;
    sens->V[1] = VIcp37_138;
    sens->V[2] = VIcp37_238;
    sens->V[3] = VIcp37_338;
    sens->OM[1] = OMcp37_138;
    sens->OM[2] = OMcp37_238;
    sens->OM[3] = OMcp37_338;
    sens->A[1] = ACcp37_138;
    sens->A[2] = ACcp37_238;
    sens->A[3] = ACcp37_338;
    sens->OMP[1] = OPcp37_138;
    sens->OMP[2] = OPcp37_238;
    sens->OMP[3] = OPcp37_338;
 
// 
break;
case 39:
 


// = = Block_1_0_0_39_0_1 = = 
 
// Sensor Kinematics 


    ROcp38_45 = -S4*C5;
    ROcp38_55 = C4*C5;
    ROcp38_75 = S4*S5;
    ROcp38_85 = -C4*S5;
    ROcp38_16 = -(ROcp38_75*S6-C4*C6);
    ROcp38_26 = -(ROcp38_85*S6-S4*C6);
    ROcp38_76 = ROcp38_75*C6+C4*S6;
    ROcp38_86 = ROcp38_85*C6+S4*S6;
    OMcp38_15 = qd[5]*C4;
    OMcp38_25 = qd[5]*S4;
    OMcp38_16 = OMcp38_15+ROcp38_45*qd[6];
    OMcp38_26 = OMcp38_25+ROcp38_55*qd[6];
    OMcp38_36 = qd[4]+qd[6]*S5;
    OPcp38_16 = ROcp38_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp38_25*S5-ROcp38_55*qd[4]);
    OPcp38_26 = ROcp38_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp38_15*S5-ROcp38_45*qd[4]);
    OPcp38_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_39_0_16 = = 
 
// Sensor Kinematics 


    ROcp38_138 = ROcp38_16*C38-ROcp38_76*S38;
    ROcp38_238 = ROcp38_26*C38-ROcp38_86*S38;
    ROcp38_738 = ROcp38_16*S38+ROcp38_76*C38;
    ROcp38_838 = ROcp38_26*S38+ROcp38_86*C38;
    RLcp38_138 = ROcp38_16*s->dpt[1][15]+ROcp38_76*s->dpt[3][15];
    RLcp38_238 = ROcp38_26*s->dpt[1][15]+ROcp38_86*s->dpt[3][15];
    RLcp38_338 = -C5*(s->dpt[1][15]*S6-s->dpt[3][15]*C6);
    POcp38_138 = RLcp38_138+q[1];
    POcp38_238 = RLcp38_238+q[2];
    POcp38_338 = RLcp38_338+q[3];
    OMcp38_138 = OMcp38_16+ROcp38_45*qd[38];
    OMcp38_238 = OMcp38_26+ROcp38_55*qd[38];
    OMcp38_338 = OMcp38_36+qd[38]*S5;
    ORcp38_138 = OMcp38_26*RLcp38_338-OMcp38_36*RLcp38_238;
    ORcp38_238 = -(OMcp38_16*RLcp38_338-OMcp38_36*RLcp38_138);
    ORcp38_338 = OMcp38_16*RLcp38_238-OMcp38_26*RLcp38_138;
    VIcp38_138 = ORcp38_138+qd[1];
    VIcp38_238 = ORcp38_238+qd[2];
    VIcp38_338 = ORcp38_338+qd[3];
    ACcp38_138 = qdd[1]+OMcp38_26*ORcp38_338-OMcp38_36*ORcp38_238+OPcp38_26*RLcp38_338-OPcp38_36*RLcp38_238;
    ACcp38_238 = qdd[2]-OMcp38_16*ORcp38_338+OMcp38_36*ORcp38_138-OPcp38_16*RLcp38_338+OPcp38_36*RLcp38_138;
    ACcp38_338 = qdd[3]+OMcp38_16*ORcp38_238-OMcp38_26*ORcp38_138+OPcp38_16*RLcp38_238-OPcp38_26*RLcp38_138;

// = = Block_1_0_0_39_0_17 = = 
 
// Sensor Kinematics 


    ROcp38_139 = ROcp38_138*C39-ROcp38_738*S39;
    ROcp38_239 = ROcp38_238*C39-ROcp38_838*S39;
    ROcp38_339 = -S38p6p39*C5;
    ROcp38_739 = ROcp38_138*S39+ROcp38_738*C39;
    ROcp38_839 = ROcp38_238*S39+ROcp38_838*C39;
    ROcp38_939 = C38p6p39*C5;
    OMcp38_139 = OMcp38_138+ROcp38_45*qd[39];
    OMcp38_239 = OMcp38_238+ROcp38_55*qd[39];
    OMcp38_339 = OMcp38_338+qd[39]*S5;
    OPcp38_139 = OPcp38_16+ROcp38_45*qdd[38]+ROcp38_45*qdd[39]+qd[38]*(OMcp38_26*S5-OMcp38_36*ROcp38_55)+qd[39]*(
 OMcp38_238*S5-OMcp38_338*ROcp38_55);
    OPcp38_239 = OPcp38_26+ROcp38_55*qdd[38]+ROcp38_55*qdd[39]-qd[38]*(OMcp38_16*S5-OMcp38_36*ROcp38_45)-qd[39]*(
 OMcp38_138*S5-OMcp38_338*ROcp38_45);
    OPcp38_339 = OPcp38_36+qdd[38]*S5+qdd[39]*S5+qd[38]*(OMcp38_16*ROcp38_55-OMcp38_26*ROcp38_45)+qd[39]*(OMcp38_138*
 ROcp38_55-OMcp38_238*ROcp38_45);

// = = Block_1_0_0_39_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp38_138;
    sens->P[2] = POcp38_238;
    sens->P[3] = POcp38_338;
    sens->R[1][1] = ROcp38_139;
    sens->R[1][2] = ROcp38_239;
    sens->R[1][3] = ROcp38_339;
    sens->R[2][1] = ROcp38_45;
    sens->R[2][2] = ROcp38_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp38_739;
    sens->R[3][2] = ROcp38_839;
    sens->R[3][3] = ROcp38_939;
    sens->V[1] = VIcp38_138;
    sens->V[2] = VIcp38_238;
    sens->V[3] = VIcp38_338;
    sens->OM[1] = OMcp38_139;
    sens->OM[2] = OMcp38_239;
    sens->OM[3] = OMcp38_339;
    sens->A[1] = ACcp38_138;
    sens->A[2] = ACcp38_238;
    sens->A[3] = ACcp38_338;
    sens->OMP[1] = OPcp38_139;
    sens->OMP[2] = OPcp38_239;
    sens->OMP[3] = OPcp38_339;
 
// 
break;
case 40:
 


// = = Block_1_0_0_40_0_1 = = 
 
// Sensor Kinematics 


    ROcp39_45 = -S4*C5;
    ROcp39_55 = C4*C5;
    ROcp39_75 = S4*S5;
    ROcp39_85 = -C4*S5;
    ROcp39_16 = -(ROcp39_75*S6-C4*C6);
    ROcp39_26 = -(ROcp39_85*S6-S4*C6);
    ROcp39_76 = ROcp39_75*C6+C4*S6;
    ROcp39_86 = ROcp39_85*C6+S4*S6;
    OMcp39_15 = qd[5]*C4;
    OMcp39_25 = qd[5]*S4;
    OMcp39_16 = OMcp39_15+ROcp39_45*qd[6];
    OMcp39_26 = OMcp39_25+ROcp39_55*qd[6];
    OMcp39_36 = qd[4]+qd[6]*S5;
    OPcp39_16 = ROcp39_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp39_25*S5-ROcp39_55*qd[4]);
    OPcp39_26 = ROcp39_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp39_15*S5-ROcp39_45*qd[4]);
    OPcp39_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_40_0_16 = = 
 
// Sensor Kinematics 


    ROcp39_138 = ROcp39_16*C38-ROcp39_76*S38;
    ROcp39_238 = ROcp39_26*C38-ROcp39_86*S38;
    ROcp39_738 = ROcp39_16*S38+ROcp39_76*C38;
    ROcp39_838 = ROcp39_26*S38+ROcp39_86*C38;
    RLcp39_138 = ROcp39_16*s->dpt[1][15]+ROcp39_76*s->dpt[3][15];
    RLcp39_238 = ROcp39_26*s->dpt[1][15]+ROcp39_86*s->dpt[3][15];
    RLcp39_338 = -C5*(s->dpt[1][15]*S6-s->dpt[3][15]*C6);
    OMcp39_138 = OMcp39_16+ROcp39_45*qd[38];
    OMcp39_238 = OMcp39_26+ROcp39_55*qd[38];
    OMcp39_338 = OMcp39_36+qd[38]*S5;
    ORcp39_138 = OMcp39_26*RLcp39_338-OMcp39_36*RLcp39_238;
    ORcp39_238 = -(OMcp39_16*RLcp39_338-OMcp39_36*RLcp39_138);
    ORcp39_338 = OMcp39_16*RLcp39_238-OMcp39_26*RLcp39_138;

// = = Block_1_0_0_40_0_17 = = 
 
// Sensor Kinematics 


    ROcp39_139 = ROcp39_138*C39-ROcp39_738*S39;
    ROcp39_239 = ROcp39_238*C39-ROcp39_838*S39;
    ROcp39_339 = -S38p6p39*C5;
    ROcp39_739 = ROcp39_138*S39+ROcp39_738*C39;
    ROcp39_839 = ROcp39_238*S39+ROcp39_838*C39;
    ROcp39_939 = C38p6p39*C5;
    ROcp39_440 = ROcp39_45*C40+ROcp39_739*S40;
    ROcp39_540 = ROcp39_55*C40+ROcp39_839*S40;
    ROcp39_640 = ROcp39_939*S40+C40*S5;
    ROcp39_740 = -(ROcp39_45*S40-ROcp39_739*C40);
    ROcp39_840 = -(ROcp39_55*S40-ROcp39_839*C40);
    ROcp39_940 = ROcp39_939*C40-S40*S5;
    OMcp39_139 = OMcp39_138+ROcp39_45*qd[39];
    OMcp39_239 = OMcp39_238+ROcp39_55*qd[39];
    OMcp39_339 = OMcp39_338+qd[39]*S5;
    OPcp39_139 = OPcp39_16+ROcp39_45*qdd[38]+ROcp39_45*qdd[39]+qd[38]*(OMcp39_26*S5-OMcp39_36*ROcp39_55)+qd[39]*(
 OMcp39_238*S5-OMcp39_338*ROcp39_55);
    OPcp39_239 = OPcp39_26+ROcp39_55*qdd[38]+ROcp39_55*qdd[39]-qd[38]*(OMcp39_16*S5-OMcp39_36*ROcp39_45)-qd[39]*(
 OMcp39_138*S5-OMcp39_338*ROcp39_45);
    OPcp39_339 = OPcp39_36+qdd[38]*S5+qdd[39]*S5+qd[38]*(OMcp39_16*ROcp39_55-OMcp39_26*ROcp39_45)+qd[39]*(OMcp39_138*
 ROcp39_55-OMcp39_238*ROcp39_45);
    RLcp39_140 = ROcp39_139*s->dpt[1][57]+ROcp39_45*s->dpt[2][57];
    RLcp39_240 = ROcp39_239*s->dpt[1][57]+ROcp39_55*s->dpt[2][57];
    RLcp39_340 = ROcp39_339*s->dpt[1][57]+s->dpt[2][57]*S5;
    POcp39_140 = RLcp39_138+RLcp39_140+q[1];
    POcp39_240 = RLcp39_238+RLcp39_240+q[2];
    POcp39_340 = RLcp39_338+RLcp39_340+q[3];
    OMcp39_140 = OMcp39_139+ROcp39_139*qd[40];
    OMcp39_240 = OMcp39_239+ROcp39_239*qd[40];
    OMcp39_340 = OMcp39_339+ROcp39_339*qd[40];
    ORcp39_140 = OMcp39_239*RLcp39_340-OMcp39_339*RLcp39_240;
    ORcp39_240 = -(OMcp39_139*RLcp39_340-OMcp39_339*RLcp39_140);
    ORcp39_340 = OMcp39_139*RLcp39_240-OMcp39_239*RLcp39_140;
    VIcp39_140 = ORcp39_138+ORcp39_140+qd[1];
    VIcp39_240 = ORcp39_238+ORcp39_240+qd[2];
    VIcp39_340 = ORcp39_338+ORcp39_340+qd[3];
    OPcp39_140 = OPcp39_139+ROcp39_139*qdd[40]+qd[40]*(OMcp39_239*ROcp39_339-OMcp39_339*ROcp39_239);
    OPcp39_240 = OPcp39_239+ROcp39_239*qdd[40]-qd[40]*(OMcp39_139*ROcp39_339-OMcp39_339*ROcp39_139);
    OPcp39_340 = OPcp39_339+ROcp39_339*qdd[40]+qd[40]*(OMcp39_139*ROcp39_239-OMcp39_239*ROcp39_139);
    ACcp39_140 = qdd[1]+OMcp39_239*ORcp39_340+OMcp39_26*ORcp39_338-OMcp39_339*ORcp39_240-OMcp39_36*ORcp39_238+OPcp39_239*
 RLcp39_340+OPcp39_26*RLcp39_338-OPcp39_339*RLcp39_240-OPcp39_36*RLcp39_238;
    ACcp39_240 = qdd[2]-OMcp39_139*ORcp39_340-OMcp39_16*ORcp39_338+OMcp39_339*ORcp39_140+OMcp39_36*ORcp39_138-OPcp39_139*
 RLcp39_340-OPcp39_16*RLcp39_338+OPcp39_339*RLcp39_140+OPcp39_36*RLcp39_138;
    ACcp39_340 = qdd[3]+OMcp39_139*ORcp39_240+OMcp39_16*ORcp39_238-OMcp39_239*ORcp39_140-OMcp39_26*ORcp39_138+OPcp39_139*
 RLcp39_240+OPcp39_16*RLcp39_238-OPcp39_239*RLcp39_140-OPcp39_26*RLcp39_138;

// = = Block_1_0_0_40_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp39_140;
    sens->P[2] = POcp39_240;
    sens->P[3] = POcp39_340;
    sens->R[1][1] = ROcp39_139;
    sens->R[1][2] = ROcp39_239;
    sens->R[1][3] = ROcp39_339;
    sens->R[2][1] = ROcp39_440;
    sens->R[2][2] = ROcp39_540;
    sens->R[2][3] = ROcp39_640;
    sens->R[3][1] = ROcp39_740;
    sens->R[3][2] = ROcp39_840;
    sens->R[3][3] = ROcp39_940;
    sens->V[1] = VIcp39_140;
    sens->V[2] = VIcp39_240;
    sens->V[3] = VIcp39_340;
    sens->OM[1] = OMcp39_140;
    sens->OM[2] = OMcp39_240;
    sens->OM[3] = OMcp39_340;
    sens->A[1] = ACcp39_140;
    sens->A[2] = ACcp39_240;
    sens->A[3] = ACcp39_340;
    sens->OMP[1] = OPcp39_140;
    sens->OMP[2] = OPcp39_240;
    sens->OMP[3] = OPcp39_340;
 
// 
break;
case 41:
 


// = = Block_1_0_0_41_0_1 = = 
 
// Sensor Kinematics 


    ROcp40_45 = -S4*C5;
    ROcp40_55 = C4*C5;
    ROcp40_75 = S4*S5;
    ROcp40_85 = -C4*S5;
    ROcp40_16 = -(ROcp40_75*S6-C4*C6);
    ROcp40_26 = -(ROcp40_85*S6-S4*C6);
    ROcp40_76 = ROcp40_75*C6+C4*S6;
    ROcp40_86 = ROcp40_85*C6+S4*S6;
    OMcp40_15 = qd[5]*C4;
    OMcp40_25 = qd[5]*S4;
    OMcp40_16 = OMcp40_15+ROcp40_45*qd[6];
    OMcp40_26 = OMcp40_25+ROcp40_55*qd[6];
    OMcp40_36 = qd[4]+qd[6]*S5;
    OPcp40_16 = ROcp40_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp40_25*S5-ROcp40_55*qd[4]);
    OPcp40_26 = ROcp40_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp40_15*S5-ROcp40_45*qd[4]);
    OPcp40_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_41_0_16 = = 
 
// Sensor Kinematics 


    ROcp40_138 = ROcp40_16*C38-ROcp40_76*S38;
    ROcp40_238 = ROcp40_26*C38-ROcp40_86*S38;
    ROcp40_738 = ROcp40_16*S38+ROcp40_76*C38;
    ROcp40_838 = ROcp40_26*S38+ROcp40_86*C38;
    RLcp40_138 = ROcp40_16*s->dpt[1][15]+ROcp40_76*s->dpt[3][15];
    RLcp40_238 = ROcp40_26*s->dpt[1][15]+ROcp40_86*s->dpt[3][15];
    RLcp40_338 = -C5*(s->dpt[1][15]*S6-s->dpt[3][15]*C6);
    OMcp40_138 = OMcp40_16+ROcp40_45*qd[38];
    OMcp40_238 = OMcp40_26+ROcp40_55*qd[38];
    OMcp40_338 = OMcp40_36+qd[38]*S5;
    ORcp40_138 = OMcp40_26*RLcp40_338-OMcp40_36*RLcp40_238;
    ORcp40_238 = -(OMcp40_16*RLcp40_338-OMcp40_36*RLcp40_138);
    ORcp40_338 = OMcp40_16*RLcp40_238-OMcp40_26*RLcp40_138;

// = = Block_1_0_0_41_0_17 = = 
 
// Sensor Kinematics 


    ROcp40_139 = ROcp40_138*C39-ROcp40_738*S39;
    ROcp40_239 = ROcp40_238*C39-ROcp40_838*S39;
    ROcp40_339 = -S38p6p39*C5;
    ROcp40_739 = ROcp40_138*S39+ROcp40_738*C39;
    ROcp40_839 = ROcp40_238*S39+ROcp40_838*C39;
    ROcp40_939 = C38p6p39*C5;
    ROcp40_440 = ROcp40_45*C40+ROcp40_739*S40;
    ROcp40_540 = ROcp40_55*C40+ROcp40_839*S40;
    ROcp40_640 = ROcp40_939*S40+C40*S5;
    ROcp40_740 = -(ROcp40_45*S40-ROcp40_739*C40);
    ROcp40_840 = -(ROcp40_55*S40-ROcp40_839*C40);
    ROcp40_940 = ROcp40_939*C40-S40*S5;
    ROcp40_141 = ROcp40_139*C41-ROcp40_740*S41;
    ROcp40_241 = ROcp40_239*C41-ROcp40_840*S41;
    ROcp40_341 = ROcp40_339*C41-ROcp40_940*S41;
    ROcp40_741 = ROcp40_139*S41+ROcp40_740*C41;
    ROcp40_841 = ROcp40_239*S41+ROcp40_840*C41;
    ROcp40_941 = ROcp40_339*S41+ROcp40_940*C41;
    OMcp40_139 = OMcp40_138+ROcp40_45*qd[39];
    OMcp40_239 = OMcp40_238+ROcp40_55*qd[39];
    OMcp40_339 = OMcp40_338+qd[39]*S5;
    OPcp40_139 = OPcp40_16+ROcp40_45*qdd[38]+ROcp40_45*qdd[39]+qd[38]*(OMcp40_26*S5-OMcp40_36*ROcp40_55)+qd[39]*(
 OMcp40_238*S5-OMcp40_338*ROcp40_55);
    OPcp40_239 = OPcp40_26+ROcp40_55*qdd[38]+ROcp40_55*qdd[39]-qd[38]*(OMcp40_16*S5-OMcp40_36*ROcp40_45)-qd[39]*(
 OMcp40_138*S5-OMcp40_338*ROcp40_45);
    OPcp40_339 = OPcp40_36+qdd[38]*S5+qdd[39]*S5+qd[38]*(OMcp40_16*ROcp40_55-OMcp40_26*ROcp40_45)+qd[39]*(OMcp40_138*
 ROcp40_55-OMcp40_238*ROcp40_45);
    RLcp40_140 = ROcp40_139*s->dpt[1][57]+ROcp40_45*s->dpt[2][57];
    RLcp40_240 = ROcp40_239*s->dpt[1][57]+ROcp40_55*s->dpt[2][57];
    RLcp40_340 = ROcp40_339*s->dpt[1][57]+s->dpt[2][57]*S5;
    POcp40_140 = RLcp40_138+RLcp40_140+q[1];
    POcp40_240 = RLcp40_238+RLcp40_240+q[2];
    POcp40_340 = RLcp40_338+RLcp40_340+q[3];
    OMcp40_140 = OMcp40_139+ROcp40_139*qd[40];
    OMcp40_240 = OMcp40_239+ROcp40_239*qd[40];
    OMcp40_340 = OMcp40_339+ROcp40_339*qd[40];
    ORcp40_140 = OMcp40_239*RLcp40_340-OMcp40_339*RLcp40_240;
    ORcp40_240 = -(OMcp40_139*RLcp40_340-OMcp40_339*RLcp40_140);
    ORcp40_340 = OMcp40_139*RLcp40_240-OMcp40_239*RLcp40_140;
    VIcp40_140 = ORcp40_138+ORcp40_140+qd[1];
    VIcp40_240 = ORcp40_238+ORcp40_240+qd[2];
    VIcp40_340 = ORcp40_338+ORcp40_340+qd[3];
    ACcp40_140 = qdd[1]+OMcp40_239*ORcp40_340+OMcp40_26*ORcp40_338-OMcp40_339*ORcp40_240-OMcp40_36*ORcp40_238+OPcp40_239*
 RLcp40_340+OPcp40_26*RLcp40_338-OPcp40_339*RLcp40_240-OPcp40_36*RLcp40_238;
    ACcp40_240 = qdd[2]-OMcp40_139*ORcp40_340-OMcp40_16*ORcp40_338+OMcp40_339*ORcp40_140+OMcp40_36*ORcp40_138-OPcp40_139*
 RLcp40_340-OPcp40_16*RLcp40_338+OPcp40_339*RLcp40_140+OPcp40_36*RLcp40_138;
    ACcp40_340 = qdd[3]+OMcp40_139*ORcp40_240+OMcp40_16*ORcp40_238-OMcp40_239*ORcp40_140-OMcp40_26*ORcp40_138+OPcp40_139*
 RLcp40_240+OPcp40_16*RLcp40_238-OPcp40_239*RLcp40_140-OPcp40_26*RLcp40_138;
    OMcp40_141 = OMcp40_140+ROcp40_440*qd[41];
    OMcp40_241 = OMcp40_240+ROcp40_540*qd[41];
    OMcp40_341 = OMcp40_340+ROcp40_640*qd[41];
    OPcp40_141 = OPcp40_139+ROcp40_139*qdd[40]+ROcp40_440*qdd[41]+qd[40]*(OMcp40_239*ROcp40_339-OMcp40_339*ROcp40_239)+
 qd[41]*(OMcp40_240*ROcp40_640-OMcp40_340*ROcp40_540);
    OPcp40_241 = OPcp40_239+ROcp40_239*qdd[40]+ROcp40_540*qdd[41]-qd[40]*(OMcp40_139*ROcp40_339-OMcp40_339*ROcp40_139)-
 qd[41]*(OMcp40_140*ROcp40_640-OMcp40_340*ROcp40_440);
    OPcp40_341 = OPcp40_339+ROcp40_339*qdd[40]+ROcp40_640*qdd[41]+qd[40]*(OMcp40_139*ROcp40_239-OMcp40_239*ROcp40_139)+
 qd[41]*(OMcp40_140*ROcp40_540-OMcp40_240*ROcp40_440);

// = = Block_1_0_0_41_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp40_140;
    sens->P[2] = POcp40_240;
    sens->P[3] = POcp40_340;
    sens->R[1][1] = ROcp40_141;
    sens->R[1][2] = ROcp40_241;
    sens->R[1][3] = ROcp40_341;
    sens->R[2][1] = ROcp40_440;
    sens->R[2][2] = ROcp40_540;
    sens->R[2][3] = ROcp40_640;
    sens->R[3][1] = ROcp40_741;
    sens->R[3][2] = ROcp40_841;
    sens->R[3][3] = ROcp40_941;
    sens->V[1] = VIcp40_140;
    sens->V[2] = VIcp40_240;
    sens->V[3] = VIcp40_340;
    sens->OM[1] = OMcp40_141;
    sens->OM[2] = OMcp40_241;
    sens->OM[3] = OMcp40_341;
    sens->A[1] = ACcp40_140;
    sens->A[2] = ACcp40_240;
    sens->A[3] = ACcp40_340;
    sens->OMP[1] = OPcp40_141;
    sens->OMP[2] = OPcp40_241;
    sens->OMP[3] = OPcp40_341;
 
// 
break;
case 42:
 


// = = Block_1_0_0_42_0_1 = = 
 
// Sensor Kinematics 


    ROcp41_45 = -S4*C5;
    ROcp41_55 = C4*C5;
    ROcp41_75 = S4*S5;
    ROcp41_85 = -C4*S5;
    ROcp41_16 = -(ROcp41_75*S6-C4*C6);
    ROcp41_26 = -(ROcp41_85*S6-S4*C6);
    ROcp41_76 = ROcp41_75*C6+C4*S6;
    ROcp41_86 = ROcp41_85*C6+S4*S6;
    OMcp41_15 = qd[5]*C4;
    OMcp41_25 = qd[5]*S4;
    OMcp41_16 = OMcp41_15+ROcp41_45*qd[6];
    OMcp41_26 = OMcp41_25+ROcp41_55*qd[6];
    OMcp41_36 = qd[4]+qd[6]*S5;
    OPcp41_16 = ROcp41_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp41_25*S5-ROcp41_55*qd[4]);
    OPcp41_26 = ROcp41_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp41_15*S5-ROcp41_45*qd[4]);
    OPcp41_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_42_0_16 = = 
 
// Sensor Kinematics 


    ROcp41_138 = ROcp41_16*C38-ROcp41_76*S38;
    ROcp41_238 = ROcp41_26*C38-ROcp41_86*S38;
    ROcp41_338 = -S38p6*C5;
    ROcp41_738 = ROcp41_16*S38+ROcp41_76*C38;
    ROcp41_838 = ROcp41_26*S38+ROcp41_86*C38;
    ROcp41_938 = C38p6*C5;
    RLcp41_138 = ROcp41_16*s->dpt[1][15]+ROcp41_76*s->dpt[3][15];
    RLcp41_238 = ROcp41_26*s->dpt[1][15]+ROcp41_86*s->dpt[3][15];
    RLcp41_338 = -C5*(s->dpt[1][15]*S6-s->dpt[3][15]*C6);
    OMcp41_138 = OMcp41_16+ROcp41_45*qd[38];
    OMcp41_238 = OMcp41_26+ROcp41_55*qd[38];
    OMcp41_338 = OMcp41_36+qd[38]*S5;
    ORcp41_138 = OMcp41_26*RLcp41_338-OMcp41_36*RLcp41_238;
    ORcp41_238 = -(OMcp41_16*RLcp41_338-OMcp41_36*RLcp41_138);
    ORcp41_338 = OMcp41_16*RLcp41_238-OMcp41_26*RLcp41_138;
    OPcp41_138 = OPcp41_16+ROcp41_45*qdd[38]+qd[38]*(OMcp41_26*S5-OMcp41_36*ROcp41_55);
    OPcp41_238 = OPcp41_26+ROcp41_55*qdd[38]-qd[38]*(OMcp41_16*S5-OMcp41_36*ROcp41_45);
    OPcp41_338 = OPcp41_36+qdd[38]*S5+qd[38]*(OMcp41_16*ROcp41_55-OMcp41_26*ROcp41_45);

// = = Block_1_0_0_42_0_18 = = 
 
// Sensor Kinematics 


    ROcp41_442 = ROcp41_45*C42+ROcp41_738*S42;
    ROcp41_542 = ROcp41_55*C42+ROcp41_838*S42;
    ROcp41_642 = ROcp41_938*S42+C42*S5;
    ROcp41_742 = -(ROcp41_45*S42-ROcp41_738*C42);
    ROcp41_842 = -(ROcp41_55*S42-ROcp41_838*C42);
    ROcp41_942 = ROcp41_938*C42-S42*S5;
    RLcp41_142 = ROcp41_138*s->dpt[1][56]+ROcp41_45*s->dpt[2][56];
    RLcp41_242 = ROcp41_238*s->dpt[1][56]+ROcp41_55*s->dpt[2][56];
    RLcp41_342 = ROcp41_338*s->dpt[1][56]+s->dpt[2][56]*S5;
    POcp41_142 = RLcp41_138+RLcp41_142+q[1];
    POcp41_242 = RLcp41_238+RLcp41_242+q[2];
    POcp41_342 = RLcp41_338+RLcp41_342+q[3];
    OMcp41_142 = OMcp41_138+ROcp41_138*qd[42];
    OMcp41_242 = OMcp41_238+ROcp41_238*qd[42];
    OMcp41_342 = OMcp41_338+ROcp41_338*qd[42];
    ORcp41_142 = OMcp41_238*RLcp41_342-OMcp41_338*RLcp41_242;
    ORcp41_242 = -(OMcp41_138*RLcp41_342-OMcp41_338*RLcp41_142);
    ORcp41_342 = OMcp41_138*RLcp41_242-OMcp41_238*RLcp41_142;
    VIcp41_142 = ORcp41_138+ORcp41_142+qd[1];
    VIcp41_242 = ORcp41_238+ORcp41_242+qd[2];
    VIcp41_342 = ORcp41_338+ORcp41_342+qd[3];
    OPcp41_142 = OPcp41_138+ROcp41_138*qdd[42]+qd[42]*(OMcp41_238*ROcp41_338-OMcp41_338*ROcp41_238);
    OPcp41_242 = OPcp41_238+ROcp41_238*qdd[42]-qd[42]*(OMcp41_138*ROcp41_338-OMcp41_338*ROcp41_138);
    OPcp41_342 = OPcp41_338+ROcp41_338*qdd[42]+qd[42]*(OMcp41_138*ROcp41_238-OMcp41_238*ROcp41_138);
    ACcp41_142 = qdd[1]+OMcp41_238*ORcp41_342+OMcp41_26*ORcp41_338-OMcp41_338*ORcp41_242-OMcp41_36*ORcp41_238+OPcp41_238*
 RLcp41_342+OPcp41_26*RLcp41_338-OPcp41_338*RLcp41_242-OPcp41_36*RLcp41_238;
    ACcp41_242 = qdd[2]-OMcp41_138*ORcp41_342-OMcp41_16*ORcp41_338+OMcp41_338*ORcp41_142+OMcp41_36*ORcp41_138-OPcp41_138*
 RLcp41_342-OPcp41_16*RLcp41_338+OPcp41_338*RLcp41_142+OPcp41_36*RLcp41_138;
    ACcp41_342 = qdd[3]+OMcp41_138*ORcp41_242+OMcp41_16*ORcp41_238-OMcp41_238*ORcp41_142-OMcp41_26*ORcp41_138+OPcp41_138*
 RLcp41_242+OPcp41_16*RLcp41_238-OPcp41_238*RLcp41_142-OPcp41_26*RLcp41_138;

// = = Block_1_0_0_42_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp41_142;
    sens->P[2] = POcp41_242;
    sens->P[3] = POcp41_342;
    sens->R[1][1] = ROcp41_138;
    sens->R[1][2] = ROcp41_238;
    sens->R[1][3] = ROcp41_338;
    sens->R[2][1] = ROcp41_442;
    sens->R[2][2] = ROcp41_542;
    sens->R[2][3] = ROcp41_642;
    sens->R[3][1] = ROcp41_742;
    sens->R[3][2] = ROcp41_842;
    sens->R[3][3] = ROcp41_942;
    sens->V[1] = VIcp41_142;
    sens->V[2] = VIcp41_242;
    sens->V[3] = VIcp41_342;
    sens->OM[1] = OMcp41_142;
    sens->OM[2] = OMcp41_242;
    sens->OM[3] = OMcp41_342;
    sens->A[1] = ACcp41_142;
    sens->A[2] = ACcp41_242;
    sens->A[3] = ACcp41_342;
    sens->OMP[1] = OPcp41_142;
    sens->OMP[2] = OPcp41_242;
    sens->OMP[3] = OPcp41_342;
 
// 
break;
case 43:
 


// = = Block_1_0_0_43_0_1 = = 
 
// Sensor Kinematics 


    ROcp42_45 = -S4*C5;
    ROcp42_55 = C4*C5;
    ROcp42_75 = S4*S5;
    ROcp42_85 = -C4*S5;
    ROcp42_16 = -(ROcp42_75*S6-C4*C6);
    ROcp42_26 = -(ROcp42_85*S6-S4*C6);
    ROcp42_76 = ROcp42_75*C6+C4*S6;
    ROcp42_86 = ROcp42_85*C6+S4*S6;
    OMcp42_15 = qd[5]*C4;
    OMcp42_25 = qd[5]*S4;
    OMcp42_16 = OMcp42_15+ROcp42_45*qd[6];
    OMcp42_26 = OMcp42_25+ROcp42_55*qd[6];
    OMcp42_36 = qd[4]+qd[6]*S5;
    OPcp42_16 = ROcp42_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp42_25*S5-ROcp42_55*qd[4]);
    OPcp42_26 = ROcp42_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp42_15*S5-ROcp42_45*qd[4]);
    OPcp42_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_43_0_16 = = 
 
// Sensor Kinematics 


    ROcp42_138 = ROcp42_16*C38-ROcp42_76*S38;
    ROcp42_238 = ROcp42_26*C38-ROcp42_86*S38;
    ROcp42_338 = -S38p6*C5;
    ROcp42_738 = ROcp42_16*S38+ROcp42_76*C38;
    ROcp42_838 = ROcp42_26*S38+ROcp42_86*C38;
    ROcp42_938 = C38p6*C5;
    RLcp42_138 = ROcp42_16*s->dpt[1][15]+ROcp42_76*s->dpt[3][15];
    RLcp42_238 = ROcp42_26*s->dpt[1][15]+ROcp42_86*s->dpt[3][15];
    RLcp42_338 = -C5*(s->dpt[1][15]*S6-s->dpt[3][15]*C6);
    OMcp42_138 = OMcp42_16+ROcp42_45*qd[38];
    OMcp42_238 = OMcp42_26+ROcp42_55*qd[38];
    OMcp42_338 = OMcp42_36+qd[38]*S5;
    ORcp42_138 = OMcp42_26*RLcp42_338-OMcp42_36*RLcp42_238;
    ORcp42_238 = -(OMcp42_16*RLcp42_338-OMcp42_36*RLcp42_138);
    ORcp42_338 = OMcp42_16*RLcp42_238-OMcp42_26*RLcp42_138;
    OPcp42_138 = OPcp42_16+ROcp42_45*qdd[38]+qd[38]*(OMcp42_26*S5-OMcp42_36*ROcp42_55);
    OPcp42_238 = OPcp42_26+ROcp42_55*qdd[38]-qd[38]*(OMcp42_16*S5-OMcp42_36*ROcp42_45);
    OPcp42_338 = OPcp42_36+qdd[38]*S5+qd[38]*(OMcp42_16*ROcp42_55-OMcp42_26*ROcp42_45);

// = = Block_1_0_0_43_0_18 = = 
 
// Sensor Kinematics 


    ROcp42_442 = ROcp42_45*C42+ROcp42_738*S42;
    ROcp42_542 = ROcp42_55*C42+ROcp42_838*S42;
    ROcp42_642 = ROcp42_938*S42+C42*S5;
    ROcp42_742 = -(ROcp42_45*S42-ROcp42_738*C42);
    ROcp42_842 = -(ROcp42_55*S42-ROcp42_838*C42);
    ROcp42_942 = ROcp42_938*C42-S42*S5;
    ROcp42_143 = ROcp42_138*C43-ROcp42_742*S43;
    ROcp42_243 = ROcp42_238*C43-ROcp42_842*S43;
    ROcp42_343 = ROcp42_338*C43-ROcp42_942*S43;
    ROcp42_743 = ROcp42_138*S43+ROcp42_742*C43;
    ROcp42_843 = ROcp42_238*S43+ROcp42_842*C43;
    ROcp42_943 = ROcp42_338*S43+ROcp42_942*C43;
    RLcp42_142 = ROcp42_138*s->dpt[1][56]+ROcp42_45*s->dpt[2][56];
    RLcp42_242 = ROcp42_238*s->dpt[1][56]+ROcp42_55*s->dpt[2][56];
    RLcp42_342 = ROcp42_338*s->dpt[1][56]+s->dpt[2][56]*S5;
    POcp42_142 = RLcp42_138+RLcp42_142+q[1];
    POcp42_242 = RLcp42_238+RLcp42_242+q[2];
    POcp42_342 = RLcp42_338+RLcp42_342+q[3];
    OMcp42_142 = OMcp42_138+ROcp42_138*qd[42];
    OMcp42_242 = OMcp42_238+ROcp42_238*qd[42];
    OMcp42_342 = OMcp42_338+ROcp42_338*qd[42];
    ORcp42_142 = OMcp42_238*RLcp42_342-OMcp42_338*RLcp42_242;
    ORcp42_242 = -(OMcp42_138*RLcp42_342-OMcp42_338*RLcp42_142);
    ORcp42_342 = OMcp42_138*RLcp42_242-OMcp42_238*RLcp42_142;
    VIcp42_142 = ORcp42_138+ORcp42_142+qd[1];
    VIcp42_242 = ORcp42_238+ORcp42_242+qd[2];
    VIcp42_342 = ORcp42_338+ORcp42_342+qd[3];
    ACcp42_142 = qdd[1]+OMcp42_238*ORcp42_342+OMcp42_26*ORcp42_338-OMcp42_338*ORcp42_242-OMcp42_36*ORcp42_238+OPcp42_238*
 RLcp42_342+OPcp42_26*RLcp42_338-OPcp42_338*RLcp42_242-OPcp42_36*RLcp42_238;
    ACcp42_242 = qdd[2]-OMcp42_138*ORcp42_342-OMcp42_16*ORcp42_338+OMcp42_338*ORcp42_142+OMcp42_36*ORcp42_138-OPcp42_138*
 RLcp42_342-OPcp42_16*RLcp42_338+OPcp42_338*RLcp42_142+OPcp42_36*RLcp42_138;
    ACcp42_342 = qdd[3]+OMcp42_138*ORcp42_242+OMcp42_16*ORcp42_238-OMcp42_238*ORcp42_142-OMcp42_26*ORcp42_138+OPcp42_138*
 RLcp42_242+OPcp42_16*RLcp42_238-OPcp42_238*RLcp42_142-OPcp42_26*RLcp42_138;
    OMcp42_143 = OMcp42_142+ROcp42_442*qd[43];
    OMcp42_243 = OMcp42_242+ROcp42_542*qd[43];
    OMcp42_343 = OMcp42_342+ROcp42_642*qd[43];
    OPcp42_143 = OPcp42_138+ROcp42_138*qdd[42]+ROcp42_442*qdd[43]+qd[42]*(OMcp42_238*ROcp42_338-OMcp42_338*ROcp42_238)+
 qd[43]*(OMcp42_242*ROcp42_642-OMcp42_342*ROcp42_542);
    OPcp42_243 = OPcp42_238+ROcp42_238*qdd[42]+ROcp42_542*qdd[43]-qd[42]*(OMcp42_138*ROcp42_338-OMcp42_338*ROcp42_138)-
 qd[43]*(OMcp42_142*ROcp42_642-OMcp42_342*ROcp42_442);
    OPcp42_343 = OPcp42_338+ROcp42_338*qdd[42]+ROcp42_642*qdd[43]+qd[42]*(OMcp42_138*ROcp42_238-OMcp42_238*ROcp42_138)+
 qd[43]*(OMcp42_142*ROcp42_542-OMcp42_242*ROcp42_442);

// = = Block_1_0_0_43_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp42_142;
    sens->P[2] = POcp42_242;
    sens->P[3] = POcp42_342;
    sens->R[1][1] = ROcp42_143;
    sens->R[1][2] = ROcp42_243;
    sens->R[1][3] = ROcp42_343;
    sens->R[2][1] = ROcp42_442;
    sens->R[2][2] = ROcp42_542;
    sens->R[2][3] = ROcp42_642;
    sens->R[3][1] = ROcp42_743;
    sens->R[3][2] = ROcp42_843;
    sens->R[3][3] = ROcp42_943;
    sens->V[1] = VIcp42_142;
    sens->V[2] = VIcp42_242;
    sens->V[3] = VIcp42_342;
    sens->OM[1] = OMcp42_143;
    sens->OM[2] = OMcp42_243;
    sens->OM[3] = OMcp42_343;
    sens->A[1] = ACcp42_142;
    sens->A[2] = ACcp42_242;
    sens->A[3] = ACcp42_342;
    sens->OMP[1] = OPcp42_143;
    sens->OMP[2] = OPcp42_243;
    sens->OMP[3] = OPcp42_343;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

