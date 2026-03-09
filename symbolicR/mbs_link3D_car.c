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
//	==> Generation Date: Tue Nov 18 11:31:41 2025
//	==> using automatic loading with extension .mbs 
//
//	==> Project name: car
//
//	==> Number of joints: 43
//
//	==> Function: F27 - Link Forces (3D)
//
//	==> Git hash: fe15d6b972a9ca9ecad0489d82703366c2afa3a1
//
////

#include <math.h> 

#include "mbs_data.h"

#include "mbs_project_interface.h"

void mbs_link3D(double **frc, double **trq,
MbsData *s, double tsim)
{
#include "mbs_link3D_car.h"

double  *q, *qd, *qdd;
double **l, **dpt;

frc = s->frc;
trq = s->trq;

q = s->q;
qd = s->qd;
qdd = s->qdd;

dpt = s->dpt;
l  = s->l;

// Number of continuation lines = 0

#include "mbs_message.h"

mbs_msg("Your symbolic files seem obsolete, i.e. not up-to-date with your MBsysPad model. Please regenerate your symbolic files (MBsysPad->Tools->Generate Symbolic Files). Exiting.\n");
mbs_msg("Error raised in mbs_link3D.\n");
s->flag_stop = 1;
return;
}
