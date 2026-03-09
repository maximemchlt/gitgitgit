//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//
//
//---------------------------


#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "user_all_id.h"
#include "user_model.h"
#include "Addon_TireGroundCar/tireGroundCar.h"
#include "mbs_matrix.h"

double* user_ExtForces(double PxF[4], double RxF[4][4], 
                       double VxF[4], double OMxF[4], 
                       double AxF[4], double OMPxF[4], 
                       MbsData *mbs_data, double tsim,int ixF)
{
    double Fx=0.0, Fy=0.0, Fz=0.0;
    double Mx=0.0, My=0.0, Mz=0.0;
    double dxF[4] ={0.0, 0.0, 0.0, 0.0};


    double *SWr = mbs_data->SWr[ixF];

    // kine_wheel variables
    double pen, rz, angslip, angcamb, slip, Pcon[4], Vcon[4], Rt_ground[4][4];
    // tire/ground contact forces varibles
    double Fwhl_T[4], Mwhl_T[4], Fwhl_I[4], Mwhl_I[4];

    // default application point of the force: anchor point to which it is attached
    int idpt = 0;
    idpt = mbs_data->xfidpt[ixF];
    dxF[1] = mbs_data->dpt[1][idpt];
    dxF[2] = mbs_data->dpt[2][idpt];
    dxF[3] = mbs_data->dpt[3][idpt];

    switch (ixF) 
    {
        /* Begin of user code */
    case F_av_g_1_id:
    case F_av_d_1_id:
    case F_ar_g_1_id:
    case F_ar_d_1_id:

        pen = 0.0;
        tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, mbs_data->user_model->FrontTire.R, &pen, &rz, &angslip, &angcamb, &slip, Pcon, Vcon, Rt_ground, dxF);
        if (mbs_data->process == 1)// equil static 
        {
                Fwhl_T[1] = 0.0;
                Fwhl_T[2] = 0.0;
                Fwhl_T[3] = mbs_data->user_model->FrontTire.K*pen;
                Mwhl_T[1] = 0.0;
                Mwhl_T[2] = 0.0;
                Mwhl_T[3] = 0.0;
        }
        else if (mbs_data->process == 2) // dynamic
        {
            if (pen>0)
            {
                Fwhl_T[3] = mbs_data->user_model->FrontTire.K*pen;
                tgc_bakker_contact(Fwhl_T, Mwhl_T, angslip, angcamb, slip);
                //mbs_car_bakker_lin_0(Fwhl_R, Mwhl_R, angslip, angcamb, slip);
            }
            else
            {
                Fwhl_T[1] = 0.0;
                Fwhl_T[2] = 0.0;
                Fwhl_T[3] = 0.0;
                Mwhl_T[3] = 0.0;
            }
        }
      
        matrix_product(Rt_ground, Fwhl_T, Fwhl_I);
        matrix_product(Rt_ground, Mwhl_T, Mwhl_I);

        break;
/* End of user code */
    }

    SWr[1]= Fwhl_I[1];
    SWr[2]= Fwhl_I[2];
    SWr[3]= Fwhl_I[3];
    SWr[4]= Mwhl_I[1];
    SWr[5]= Mwhl_I[2];
    SWr[6]= Mwhl_I[3];
    SWr[7]=dxF[1];
    SWr[8]=dxF[2];
    SWr[9]=dxF[3];

    return SWr;
}

 
