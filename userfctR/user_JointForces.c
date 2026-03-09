//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h" 

#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

#include "user_all_id.h"
#include "user_model.h"

double* user_JointForces(MbsData *mbs_data, double tsim)
{
/*-- Begin of user code --*/


        // equilibrium process !
    if(mbs_data->tsim==0.0)
    {   mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQ.Qpropulsion;
        mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQ.Qpropulsion;
        mbs_data->Qq[T2_rack_id]= mbs_data->user_model->EquilQ.Qsteer;
    }


        // anti-roll bar
        mbs_data->Qq[R2_def_bar_ft_id] = -mbs_data->user_model->FrontSuspension.C_bar*mbs_data->q[R2_def_bar_ft_id];
        mbs_data->Qq[R2_def_bar_rr_id] = -mbs_data->user_model->RearSuspension.C_bar*mbs_data->q[R2_def_bar_rr_id];






    
    if (tsim > .5 && tsim < 5.0)
    {
        mbs_data->Qq[T2_rack_id] = -400;
    }
    else
    {
        mbs_data->Qq[T2_rack_id] = 0.0;
    }

/*-- End of user code --*/

    return mbs_data->Qq;
}
