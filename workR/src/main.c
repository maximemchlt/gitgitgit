   /**
    *
    *   Universite catholique de Louvain
    *   CEREM : Centre for research in mechatronics
    *   http://www.robotran.be
    *   Contact : info@robotran.be
    *
    *
    *   MBsysC main script template for simple model:
    *   -----------------------------------------------
    *    This template loads the data file *.mbs and execute:
    *      - the coordinate partitioning module
    *      - the direct dynamic module (time integration of
    *        equations of motion).
    *    It may be adapted and completed by the user.
    *
    *    (c) Universite catholique de Louvain
    *
    * To turn this file as a C++ file, just change its extension to .cc (or .cpp).
    * If you plan to use some C++ files, it is usually better that the main is compiled as a C++ function.
    * Currently, most compilers do not require this, but it is a safer approach to port your code to other computers.
    */

#include <stdio.h>
#include "mbs_data.h"
#include "mbs_dirdyn.h"
#include "mbs_part.h"
//#include "realtime.h"
#include "mbs_set.h"
#include "mbs_loader.h"
//#include "cmake_config.h"

#include "mbs_equil.h"
#include "mbs_modal.h"
#include "user_all_id.h"
#include "user_model.h"

#include "mbs_set.h"

int main(int argc, char const *argv[])
{

    MbsData *mbs_data;

    MbsPart *mbs_part;
    MbsDirdyn *mbs_dirdyn;
    MbsEquil *mbs_equil;
    MbsModal *mbs_modal;

    double V, R;

    printf("Starting car MBS project!\n");


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                     LOADING                               *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    printf("Loading the car data file !\n");
    mbs_data = mbs_load("car.mbs");
    printf("*.mbs file loaded!\n");


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *              COORDINATE PARTITIONING                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_set_qdriven(mbs_data, R1_bras_inf_av_g_1_id);
    mbs_set_qdriven(mbs_data, R1_bras_inf_av_d_1_id);
    mbs_set_qdriven(mbs_data, R1_bras_inf_ar_g_1_id);
    mbs_set_qdriven(mbs_data, R1_bras_inf_ar_d_1_id);

    mbs_part = mbs_new_part(mbs_data);

    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 1;

    mbs_run_part(mbs_part, mbs_data);
    mbs_delete_part(mbs_part);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                STATIC    EQUILIBRIUM                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_data->process = 1;
    mbs_equil = mbs_new_equil(mbs_data);

    mbs_equil->options->senstol = 1e-1;
    mbs_equil->options->devjac = 1e-2;
    mbs_equil->options->verbose = 1;
    mbs_equil->options->resfilename = "equil_static";

    mbs_run_equil(mbs_equil, mbs_data);
    mbs_delete_equil(mbs_equil, mbs_data);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                STRAIGHT LANE  EQUILIBRIUM                 *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    V = 10;
    // conditions initiales pour la vitesse de rotation des roues (approximation rsg)
    mbs_data->qd[T1_chassis_id] = V;
    mbs_data->qd[R2_wheel_ft_lt_id] = V / 0.279572;
    mbs_data->qd[R2_wheel_ft_rt_id] = V / 0.279572;
    mbs_data->qd[R2_wheel_rr_lt_id] = V / 0.280501;
    mbs_data->qd[R2_wheel_rr_rt_id] = V / 0.280501;

    mbs_equil = mbs_new_equil(mbs_data);

    mbs_equil->options->mode = 2;
    mbs_data->process = 2;
    mbs_equil->options->senstol = 1e-1;
    mbs_equil->options->devjac = 1e-6;
    mbs_equil->options->equitol = 1e-6;
    mbs_equil->options->verbose = 1;
    mbs_equil->options->resfilename = "equil_straightlane";
    // --- Variable exchange, quch->xch
    mbs_equil->options->nquch = 5;
    mbs_equil_exchange(mbs_equil->options);
    mbs_equil->options->quch[1] = T1_chassis_id;
    mbs_equil->options->xch_ptr[1] = &(mbs_data->user_model->EquilQ.Qpropulsion);
    mbs_equil->options->quch[2] = R2_wheel_ft_lt_id;
    mbs_equil->options->xch_ptr[2] = &(mbs_data->qd[R2_wheel_ft_lt_id]);
    mbs_equil->options->quch[3] = R2_wheel_ft_rt_id;
    mbs_equil->options->xch_ptr[3] = &(mbs_data->qd[R2_wheel_ft_rt_id]);
    mbs_equil->options->quch[4] = R2_wheel_rr_lt_id;
    mbs_equil->options->xch_ptr[4] = &(mbs_data->qd[R2_wheel_rr_lt_id]);
    mbs_equil->options->quch[5] = R2_wheel_rr_rt_id;
    mbs_equil->options->xch_ptr[5] = &(mbs_data->qd[R2_wheel_rr_rt_id]);

    mbs_run_equil(mbs_equil, mbs_data);
    mbs_delete_equil(mbs_equil, mbs_data);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   DIRECT DYNANMICS                        *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_data->process = 2;
    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    mbs_dirdyn->options->dt0 = 1e-3;
    mbs_dirdyn->options->tf  = 5;
    mbs_dirdyn->options->save2file = 1;

    mbs_run_dirdyn(mbs_dirdyn, mbs_data);
    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     *                   CLOSING OPERATIONS                      *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_delete_data(mbs_data);

    return 0;
}
