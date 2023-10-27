/*
 * run_pram.c
 *
 *  Created on: Aug 13, 2022
 *      Author: sato1
 */

#ifndef MODULE_INC_RUN_PRAM_C_
#define MODULE_INC_RUN_PRAM_C_

#include "typedef.h"
#include "macro.h"

const static t_turn_param sla_neta_R90_v0 = {0.5, -6.0*PI, -200*PI, 10.87, 10.87, -90.0, RIGHT};/*	float velo;float omega;float omega_acc;float Lstart;float Lend;*/
const static t_turn_param sla_neta_L90_v0 = {0.5, 6.0*PI,  200*PI,  10.87, 10.87,  90.0, LEFT };/*	float velo;float omega;float omega_acc;float Lstart;float Lend;*/

const static t_turn_param slalom_R90_search = {0.3f, -4.0f*PI, -80.0f*PI, 14.3f,14.3f,-90.0, RIGHT};
const static t_turn_param slalom_L90_search = {0.3f,  4.0f*PI,  80.0f*PI, 14.3f,14.3f, 90.0, LEFT };

//-----------search parameters
const static t_pid_gain sp_gain_search_straight = {16.0f, 0.3f, 0.0f};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_straight = {0.6f, 0.01f, 0.0f};//{0.50f, 0.0005f, 0.001f};
const static t_pid_gain sp_gain_search_turn = {16.0,0.1,0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_turn = {0.80f, 0.0005f, 0.000f};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_table = {0.30f, 25.0f,14.72,11.93+0.0, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_table = {0.30f,-25.0f,14.72,11.93+0.0,-90.0f,RIGHT};
const static t_param param_L90_search = {&slalom_L90_table ,&sp_gain_search_turn,&om_gain_search_turn};
const static t_param param_R90_search = {&slalom_R90_table, &sp_gain_search_turn,&om_gain_search_turn};


const static t_pid_gain sp_gain_search_ext_turn = {16.0,0.6,0.0};//{12.0,0.1,0.0};
const static t_pid_gain om_gain_search_ext_turn = {0.80f, 0.001f, 0.000f};//{0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_ext_table = {0.60f, 22.0f,7.69+5.0,8.33+5.0, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_ext_table = {0.60f,-22.0f,7.69+5.0,8.33+5.0,-90.0f,RIGHT};
const static t_param param_L90_ext_search = {&slalom_L90_ext_table ,&sp_gain_search_ext_turn,&om_gain_search_ext_turn};
const static t_param param_R90_ext_search = {&slalom_R90_ext_table, &sp_gain_search_ext_turn,&om_gain_search_ext_turn};



//----------straight parameters
const static t_pid_gain sp_gain_300 = {16.0f, 0.3f, 0.0f};
const static t_pid_gain om_gain_300 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_300 = {0.30f,4.0f};
const static t_straight_param st_param_300 = {&param_300,&sp_gain_300,&om_gain_300};

const static t_pid_gain sp_gain_450 = {16.0f, 0.4f, 0.0f};
const static t_pid_gain om_gain_450 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_450 = {0.45f,6.0f};
const static t_straight_param st_param_450 = {&param_450,&sp_gain_450,&om_gain_450};

const static t_pid_gain sp_gain_500 = {16.0f, 0.3f, 0.0f};
const static t_pid_gain om_gain_500 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_500 = {0.50f,6.0f};
const static t_straight_param st_param_500 = {&param_500,&sp_gain_300,&om_gain_300};

const static t_pid_gain sp_gain_600 = {16.0f, 0.4f, 0.0f};
const static t_pid_gain om_gain_600 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_600 = {0.60f,6.0f};
const static t_straight_param st_param_600 = {&param_600,&sp_gain_600,&om_gain_600};

const static t_pid_gain sp_gain_700 = {12.0,0.3,-0.2};
const static t_pid_gain om_gain_700 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_700 = {0.70f,6.0f};
const static t_straight_param st_param_700 = {&param_700,&sp_gain_700,&om_gain_700};

const static t_pid_gain sp_gain_700_no_suction = {16.0,0.4,-0.3};
const static t_pid_gain om_gain_700_no_suction  = {0.4f, 0.001f, 0.0f};
const static t_velo_param param_700_no_suction  = {0.70f,6.0f};
const static t_straight_param st_param_700_no_suction  = {&param_700_no_suction ,&sp_gain_700_no_suction ,&om_gain_700_no_suction };

const static t_pid_gain sp_gain_750 = {12.0,0.3,0.0};
const static t_pid_gain om_gain_750 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_750 = {0.75f,6.0f};
const static t_straight_param st_param_750 = {&param_750,&sp_gain_750,&om_gain_750};

const static t_pid_gain sp_gain_1000 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_1000 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_1000 = {1.00f,6.0f};
const static t_straight_param st_param_1000 = {&param_1000,&sp_gain_1000,&om_gain_1000};

const static t_pid_gain sp_gain_1000_no_suction = {12.0,0.2,0.0};
const static t_pid_gain om_gain_1000_no_suction = {0.4f, 0.001f, 0.0f};
const static t_velo_param param_1000_no_suction = {1.00f,6.0f};
const static t_straight_param st_param_1000_no_suction = {&param_1000_no_suction,&sp_gain_1000_no_suction,&om_gain_1000_no_suction};

const static t_pid_gain sp_gain_1200 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_1200 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_1200 = {1.20f,10.0f};
const static t_straight_param st_param_1200 = {&param_1200,&sp_gain_1200,&om_gain_1200};

const static t_pid_gain sp_gain_1300 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_1300 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_1300 = {1.30f,12.0f};
const static t_straight_param st_param_1300 = {&param_1300,&sp_gain_1300,&om_gain_1300};

const static t_pid_gain sp_gain_1500 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_1500 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_1500 = {1.50f,11.5f};
const static t_straight_param st_param_1500 = {&param_1500,&sp_gain_1500,&om_gain_1500};


const static t_velo_param param_1500_15 = {1.50f,15.0f};
const static t_straight_param st_param_1500_15 = {&param_1500_15,&sp_gain_1500,&om_gain_1500};

const static t_pid_gain sp_gain_1500_v1 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_1500_v1 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_1500_v1 = {1.50f,20.0f};
const static t_straight_param st_param_1500_v1 = {&param_1500_v1,&sp_gain_1500_v1,&om_gain_1500_v1};

const static t_pid_gain sp_gain_1500_v2 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_1500_v2 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_1500_v2 = {1.50f,25.0f};
const static t_straight_param st_param_1500_v2 = {&param_1500_v2,&sp_gain_1500_v2,&om_gain_1500_v2};

const static t_pid_gain sp_gain_2000 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_2000 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_2000 = {2.00f,12.0f};
const static t_straight_param st_param_2000 = {&param_2000,&sp_gain_2000,&om_gain_2000};

const static t_velo_param param_2000_15 = {2.00f,15.0f};
const static t_straight_param st_param_2000_15 = {&param_2000_15,&sp_gain_2000,&om_gain_2000};

const static t_pid_gain sp_gain_2000_v1 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_2000_v1 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_2000_v1 = {2.00f,20.0f};
const static t_straight_param st_param_2000_v1 = {&param_2000_v1,&sp_gain_2000_v1,&om_gain_2000_v1};

const static t_pid_gain sp_gain_2000_v2 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_2000_v2 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_2000_v2 = {2.00f,25.0f};
const static t_straight_param st_param_2000_v2 = {&param_2000_v2,&sp_gain_2000_v2,&om_gain_2000_v2};


const static t_pid_gain sp_gain_2500 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_2500 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_2500 = {2.50f,12.0f};
const static t_straight_param st_param_2500 = {&param_2500,&sp_gain_2500,&om_gain_2500};

const static t_velo_param param_2500_15 = {2.50f,15.0f};
const static t_straight_param st_param_2500_15 = {&param_2500_15,&sp_gain_2500,&om_gain_2500};

const static t_pid_gain sp_gain_2500_v1 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_2500_v1 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_2500_v1 = {2.50f,20.0f};
const static t_straight_param st_param_2500_v1 = {&param_2500_v1,&sp_gain_2500_v1,&om_gain_2500_v1};

const static t_pid_gain sp_gain_2500_v2 = {14.0,0.1,0.0};
const static t_pid_gain om_gain_2500_v2 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_2500_v2 = {2.50f,25.0f};
const static t_straight_param st_param_2500_v2 = {&param_2500_v2,&sp_gain_2500_v2,&om_gain_2500_v2};


const static t_pid_gain sp_gain_3000 = {14.0,0.05,0.0};
const static t_pid_gain om_gain_3000 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_3000 = {3.00f,12.8f};
const static t_straight_param st_param_3000 = {&param_3000,&sp_gain_3000,&om_gain_3000};

const static t_velo_param param_3000_15 = {3.00f,15.0f};
const static t_straight_param st_param_3000_15 = {&param_3000_15,&sp_gain_3000,&om_gain_3000};

const static t_pid_gain sp_gain_3000_v1 = {12.0,0.05,0.0};
const static t_pid_gain om_gain_3000_v1 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_3000_v1 = {3.00f,20.0};
const static t_straight_param st_param_3000_v1 = {&param_3000_v1,&sp_gain_3000_v1,&om_gain_3000_v1};

const static t_pid_gain sp_gain_3000_v2 = {12.0,0.05,0.0};
const static t_pid_gain om_gain_3000_v2 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_3000_v2 = {3.00f,25.0};
const static t_straight_param st_param_3000_v2 = {&param_3000_v2,&sp_gain_3000_v2,&om_gain_3000_v2};


const static t_pid_gain sp_gain_3500 = {14.0,0.05,0.0};
const static t_pid_gain om_gain_3500 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_3500 = {3.50f,18.50f};
const static t_straight_param st_param_3500 = {&param_3500,&sp_gain_3500,&om_gain_3500};

const static t_pid_gain sp_gain_3500_v1 = {14.0,0.05,0.0};
const static t_pid_gain om_gain_3500_v1 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_3500_v1 = {3.50f,20.0f};
const static t_straight_param st_param_3500_v1 = {&param_3500_v1,&sp_gain_3500_v1,&om_gain_3500_v1};

const static t_pid_gain sp_gain_3500_v2 = {14.0,0.05,0.0};
const static t_pid_gain om_gain_3500_v2 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_3500_v2 = {3.50f,25.0f};
const static t_straight_param st_param_3500_v2 = {&param_3500_v2,&sp_gain_3500_v2,&om_gain_3500_v2};

const static t_pid_gain sp_gain_4000 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_4000 = {0.6f, 0.01f, 0.01f};
const static t_velo_param param_4000 = {4.0f,20.0f};
const static t_straight_param st_param_4000 = {&param_4000,&sp_gain_4000,&om_gain_4000};

const static t_pid_gain sp_gain_4000_v1 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_4000_v1 = {0.6f, 0.01f, 0.0f};
const static t_velo_param param_4000_v1 = {4.0f,25.0f};
const static t_straight_param st_param_4000_v1 = {&param_4000_v1,&sp_gain_4000_v1,&om_gain_4000_v1};


const static t_straight_param *const st_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const st_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const st_mode_300_v2[] = {&st_param_300,&st_param_500,&st_param_750};
const static t_straight_param *const st_mode_300_v3[] = {&st_param_300,&st_param_500,&st_param_750,&st_param_1000};

const static t_straight_param *const di_mode_300_v0[] = {&st_param_300};
const static t_straight_param *const di_mode_300_v1[] = {&st_param_300,&st_param_500};
const static t_straight_param *const di_mode_300_v2[] = {&st_param_300,&st_param_500,&st_param_750};
const static t_straight_param *const di_mode_300_v3[] = {&st_param_300,&st_param_500,&st_param_750,&st_param_1000};

const static t_straight_param *const st_mode_500_v0[] = {&st_param_500};
const static t_straight_param *const st_mode_500_v1[] = {&st_param_500,&st_param_750};
const static t_straight_param *const st_mode_500_v2[] = {&st_param_500,&st_param_750,&st_param_1000};
const static t_straight_param *const st_mode_500_v3[] = {&st_param_500,&st_param_750,&st_param_1000,&st_param_1500,&st_param_2000};

const static t_straight_param *const di_mode_500_v0[] = {&st_param_500};
const static t_straight_param *const di_mode_500_v1[] = {&st_param_500,&st_param_750};
const static t_straight_param *const di_mode_500_v2[] = {&st_param_500,&st_param_750,&st_param_1000};
const static t_straight_param *const di_mode_500_v3[] = {&st_param_500,&st_param_750,&st_param_1000,&st_param_1500,&st_param_2000};

const static t_straight_param *const st_mode_700_v0[] = {&st_param_700,&st_param_1000};//,&st_param_1500};
const static t_straight_param *const di_mode_700_v0[] = {&st_param_700,&st_param_1000};//,&st_param_1500};

const static t_straight_param *const st_mode_700_no_suction[] = {&st_param_700_no_suction,&st_param_1000_no_suction};//,&st_param_1500};
const static t_straight_param *const di_mode_700_no_suction[] = {&st_param_700_no_suction,&st_param_1000_no_suction};//,&st_param_1500};

const static t_straight_param *const st_mode_1000_v0[] = {&st_param_1000};
const static t_straight_param *const di_mode_1000_v0[] = {&st_param_1000};

const static t_straight_param *const st_mode_1000_v1[] = {&st_param_1000,&st_param_1500,&st_param_2000,&st_param_2500,&st_param_3000};
const static t_straight_param *const di_mode_1000_v1[] = {&st_param_1000,&st_param_1500,&st_param_2000,&st_param_2500};

const static t_straight_param *const st_mode_1000_v2[] = {&st_param_1000,&st_param_1500_v1,&st_param_2000_v1,&st_param_2500_v1,&st_param_3000_v1,&st_param_3500_v1,&st_param_4000};

const static t_straight_param *const st_mode_1000_v3[] = {&st_param_1000,&st_param_1500_v2,&st_param_2000_v2,&st_param_2500_v2,&st_param_3000_v2,&st_param_3500_v2,&st_param_4000_v1};

const static t_straight_param *const st_mode_1200_v0[] = {&st_param_1200};
const static t_straight_param *const di_mode_1200_v0[] = {&st_param_1200};

const static t_straight_param *const st_mode_1200_v1[] = {&st_param_1200,&st_param_1500,&st_param_2000,&st_param_2500,&st_param_3000};
const static t_straight_param *const di_mode_1200_v1[] = {&st_param_1200,&st_param_1500,&st_param_2000,&st_param_2500};

const static t_straight_param *const st_mode_1200_v2[] = {&st_param_1200,&st_param_1500_v1,&st_param_2000_v1,&st_param_2500_v1,&st_param_3000_v1,&st_param_3500_v1,&st_param_4000};
const static t_straight_param *const di_mode_1200_v2[] = {&st_param_1200,&st_param_1500,&st_param_2000,&st_param_2500};

const static t_straight_param *const st_mode_1200_v3[] = {&st_param_1200,&st_param_1500_v2,&st_param_2000_v2,&st_param_2500_v2,&st_param_3000_v2,&st_param_3500_v2,&st_param_4000_v1};
const static t_straight_param *const di_mode_1200_v3[] = {&st_param_1200,&st_param_1500_v1,&st_param_2000_v1,&st_param_2500_v1,&st_param_3000_v1,&st_param_3500_v1,&st_param_4000};


const static t_straight_param *const st_mode_1300_v1[] = {&st_param_1300,&st_param_1500_15,&st_param_2000_15,&st_param_2500_15,&st_param_3000_15};
const static t_straight_param *const di_mode_1300_v1[] = {&st_param_1300,&st_param_1500_15,&st_param_2000_15,&st_param_2500_15};
//dummy parameters
const static t_pid_gain sp_gain_dummy = {0.0f,0.0f,0.0f};
const static t_pid_gain om_gain_dummy = {0.0f, 0.0f, 0.0f};
const static t_turn_param_table slalom_dummy = {0.0f,0.0f,0.0f,0.0f,0.0f,LEFT};
const static t_param param_dummy = {&slalom_dummy,&sp_gain_dummy,&om_gain_dummy};

//-----------velo = 300 mm/s parameters
const static t_pid_gain sp_gain_turn90_300 = {15.0,0.2,0.0};
const static t_pid_gain om_gain_turn90_300 = {0.50f, 0.0005f, 0.001f};
const static t_turn_param_table slalom_L90_300_table = {0.30f, 37.5f,39.05,39.80, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_300_table = {0.30f,-37.5f,39.05,39.80,-90.0f,RIGHT};
const static t_param param_L90_300 = {&slalom_L90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};
const static t_param param_R90_300 = {&slalom_R90_300_table,&sp_gain_turn90_300,&om_gain_turn90_300};

const static t_pid_gain sp_gain_turn180_300 = {15.0,0.2,0.0};
const static t_pid_gain om_gain_turn180_300 = {0.50f, 0.05f, 0.001f};
const static t_turn_param_table slalom_L180_300_table = {0.30f, 42.5f,23.63,24.47, 180.0f,LEFT};
const static t_turn_param_table slalom_R180_300_table = {0.30f,-42.5f,23.63,24.47,-180.0f,RIGHT};
const static t_param param_L180_300 = {&slalom_L180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};
const static t_param param_R180_300 = {&slalom_R180_300_table,&sp_gain_turn180_300,&om_gain_turn180_300};

const static t_pid_gain sp_gain_turnV90_300 = {12.0,0.2,0.0};
const static t_pid_gain om_gain_turnV90_300 = {0.8f, 0.05f, 0.00f};
const static t_turn_param_table slalom_LV90_300_table = {0.30f, 30.0f,22.78,23.50, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_300_table = {0.30f,-30.0f,22.78,23.50,-90.0f,RIGHT};
const static t_param param_LV90_300 = {&slalom_LV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};
const static t_param param_RV90_300 = {&slalom_RV90_300_table,&sp_gain_turnV90_300,&om_gain_turnV90_300};

const static t_pid_gain sp_gain_turnIn45_300 = {15.0,0.2,0.0};
const static t_pid_gain om_gain_turnIn45_300 = {0.75f, 0.01f, 0.00f};
const static t_turn_param_table slalom_inL45_300_table = {0.30f, 30.0f,27.04,46.34, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_300_table = {0.30f,-30.0f,27.04,46.34,-45.0f,RIGHT};
const static t_param param_inL45_300 = {&slalom_inL45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};
const static t_param param_inR45_300 = {&slalom_inR45_300_table,&sp_gain_turnIn45_300,&om_gain_turnIn45_300};

const static t_pid_gain sp_gain_turnOut45_300 = {15.0,0.2,0.0};
const static t_pid_gain om_gain_turnOut45_300 = {0.6f, 0.01f, 0.00f};
const static t_turn_param_table slalom_outL45_300_table = {0.30f, 30.0f,45.68,27.70, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_300_table = {0.30f,-30.0f,45.68,27.70,-45.0f,RIGHT};
const static t_param param_outL45_300 = {&slalom_outL45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};
const static t_param param_outR45_300 = {&slalom_outR45_300_table,&sp_gain_turnOut45_300,&om_gain_turnOut45_300};

const static t_pid_gain sp_gain_turnIn135_300 = {15.0,0.2,0.0};
const static t_pid_gain om_gain_turnIn135_300 = {1.5f, 0.008f, 0.0f};
const static t_turn_param_table slalom_inL135_300_table = {0.30f, 30.0f,45.29+5,38.35, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_300_table = {0.30f,-30.0f,45.29+5,38.35,-135.0f,RIGHT};
const static t_param param_inL135_300 = {&slalom_inL135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};
const static t_param param_inR135_300 = {&slalom_inR135_300_table,&sp_gain_turnIn135_300,&om_gain_turnIn135_300};

const static t_pid_gain sp_gain_turnOut135_300 = {15.0,0.2,0.0};
const static t_pid_gain om_gain_turnOut135_300 = {1.0f, 0.008f, 0.0f};
const static t_turn_param_table slalom_outL135_300_table = {0.30f, 30.0f,37.57,46.07, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_300_table = {0.30f,-30.0f,37.57,46.07,-135.0f,RIGHT};
const static t_param param_outL135_300 = {&slalom_outL135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};
const static t_param param_outR135_300 = {&slalom_outR135_300_table,&sp_gain_turnOut135_300,&om_gain_turnOut135_300};

const static t_param *const mode_300[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_300,		&param_L90_300,
												&param_R180_300,	&param_L180_300,
												&param_inR45_300,	&param_inL45_300,
												&param_outR45_300,	&param_outL45_300,
												&param_inR135_300,	&param_inL135_300,
												&param_outR135_300,	&param_outL135_300,
												&param_RV90_300,	&param_LV90_300
											};
//-----------k = 200
//-----------velo = 500 mm/s parameters
const static t_pid_gain sp_gain_turn90_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_500 = {0.8f, 0.005f, 0.00f};
const static t_turn_param_table slalom_L90_500_table = {0.50f, 42.5f,31.50+10.0,33.87+5.0, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_500_table = {0.50f,-42.5f,31.50+10.0,33.87+5.0,-90.0f,RIGHT};
const static t_param param_L90_500 = {&slalom_L90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};
const static t_param param_R90_500 = {&slalom_R90_500_table,&sp_gain_turn90_500,&om_gain_turn90_500};

const static t_pid_gain sp_gain_turn180_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_500 = {0.80f, 0.03f, 0.00f};
const static t_turn_param_table slalom_L180_500_table = {0.50f, 42.5f,22.4,25.48, 180.0f,LEFT};
const static t_turn_param_table slalom_R180_500_table = {0.50f,-42.5f,22.4,25.48,-180.0f,RIGHT};
const static t_param param_L180_500 = {&slalom_L180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};
const static t_param param_R180_500 = {&slalom_R180_500_table,&sp_gain_turn180_500,&om_gain_turn180_500};

const static t_pid_gain sp_gain_turnV90_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_500 = {0.8f, 0.01f, 0.00f};
const static t_turn_param_table slalom_LV90_500_table = {0.50f, 35.0f,15.23,17.27, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_500_table = {0.50f,-35.0f,15.23,17.27,-90.0f,RIGHT};
const static t_param param_LV90_500 = {&slalom_LV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};
const static t_param param_RV90_500 = {&slalom_RV90_500_table,&sp_gain_turnV90_500,&om_gain_turnV90_500};

const static t_pid_gain sp_gain_turnIn45_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_500 = {0.9f, 0.0005f, 0.0f};
const static t_turn_param_table slalom_inL45_500_table = {0.50f, 42.5f,18.91,39.79, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_500_table = {0.50f,-42.5f,18.91,39.79,-45.0f,RIGHT};
const static t_param param_inL45_500 = {&slalom_inL45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};
const static t_param param_inR45_500 = {&slalom_inR45_500_table,&sp_gain_turnIn45_500,&om_gain_turnIn45_500};

const static t_pid_gain sp_gain_turnOut45_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_500 = {0.9f, 0.0005f, 0.0f};
const static t_turn_param_table slalom_outL45_500_table = {0.50f, 42.5f,37.59,21.13, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_500_table = {0.50f,-42.5f,37.59,21.13,-45.0f,RIGHT};
const static t_param param_outL45_500 = {&slalom_outL45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};
const static t_param param_outR45_500 = {&slalom_outR45_500_table,&sp_gain_turnOut45_500,&om_gain_turnOut45_500};

const static t_pid_gain sp_gain_turnIn135_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_500 = {0.75f, 0.03f, 0.0f};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_500_table = {0.50f, 35.0f,29.58+4.0,24.17, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_500_table = {0.50f,-35.0f,29.58+4.0,24.17,-135.0f,RIGHT};
const static t_param param_inL135_500 = {&slalom_inL135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};
const static t_param param_inR135_500 = {&slalom_inR135_500_table,&sp_gain_turnIn135_500,&om_gain_turnIn135_500};

const static t_pid_gain sp_gain_turnOut135_500 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_500 = {0.8f, 0.08f, 0.0f};
const static t_turn_param_table slalom_outL135_500_table = {0.50f, 35.0f,21.87,31.90, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_500_table = {0.50f,-35.0f,21.87,31.90,-135.0f,RIGHT};
const static t_param param_outL135_500 = {&slalom_outL135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};
const static t_param param_outR135_500 = {&slalom_outR135_500_table,&sp_gain_turnOut135_500,&om_gain_turnOut135_500};

const static t_param *const mode_500[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_500,		&param_L90_500,
												&param_R180_500,	&param_L180_500,
												&param_inR45_500,	&param_inL45_500,
												&param_outR45_500,	&param_outL45_500,
												&param_inR135_500,	&param_inL135_500,
												&param_outR135_500,	&param_outL135_500,
												&param_RV90_500,	&param_LV90_500
											};
//k = 150
//-----------velo = 700 mm/s parameters

const static t_pid_gain sp_gain_turn90_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_700 = {0.75f, 0.0f, 0.00f};
const static t_turn_param_table slalom_L90_700_table = {0.70f, 42.5f,30.22,34.63+5.0, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_700_table = {0.70f,-42.5f,30.22,34.63+5.0,-90.0f,RIGHT};
const static t_param param_L90_700 = {&slalom_L90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700};
const static t_param param_R90_700 = {&slalom_R90_700_table,&sp_gain_turn90_700,&om_gain_turn90_700};

const static t_pid_gain sp_gain_turn180_700 = {10.0,0.4,0.0};
const static t_pid_gain om_gain_turn180_700 = {0.4f, 0.0001f, 0.00f};//{1.0f, 0.0001f, 0.00f};
const static t_turn_param_table slalom_L180_700_table = {0.70f, 45.0f,21.63,26.17, 180.0f,LEFT};
const static t_turn_param_table slalom_R180_700_table = {0.70f,-45.0f,21.63,26.17,-180.0f,RIGHT};
const static t_param param_L180_700 = {&slalom_L180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700};
const static t_param param_R180_700 = {&slalom_R180_700_table,&sp_gain_turn180_700,&om_gain_turn180_700};

const static t_pid_gain sp_gain_turnV90_700 = {10.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_700 = {0.6f, 0.0f, 0.0f};//0.8
//const static t_turn_param_table slalom_LV90_700_table = {0.70f, 35.0f,13.95,18.32, 90.0f,LEFT};
//const static t_turn_param_table slalom_RV90_700_table = {0.70f,-35.0f,13.95,18.32,-90.0f,RIGHT};
const static t_turn_param_table slalom_LV90_700_table = {0.70f, 40.0f,8.95,10.32, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_700_table = {0.70f,-40.0f,8.95,10.32,-90.0f,RIGHT};
const static t_param param_LV90_700 = {&slalom_LV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700};
const static t_param param_RV90_700 = {&slalom_RV90_700_table,&sp_gain_turnV90_700,&om_gain_turnV90_700};

const static t_pid_gain sp_gain_turnIn45_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_700 = {0.8f, 0.00f, 0.0f};
const static t_turn_param_table slalom_inL45_700_table = {0.70f, 42.5f,17.48+5.0,40.82, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_700_table = {0.70f,-42.5f,17.48+5.0,40.82,-45.0f,RIGHT};
const static t_param param_inL45_700 = {&slalom_inL45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700};
const static t_param param_inR45_700 = {&slalom_inR45_700_table,&sp_gain_turnIn45_700,&om_gain_turnIn45_700};

const static t_pid_gain sp_gain_turnOut45_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_700 = {0.8f, 0.00f, 0.0f};
const static t_turn_param_table slalom_outL45_700_table = {0.70f, 42.5f,36.38,22.00, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_700_table = {0.70f,-42.5f,36.38,22.00,-45.0f,RIGHT};
const static t_param param_outL45_700 = {&slalom_outL45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700};
const static t_param param_outR45_700 = {&slalom_outR45_700_table,&sp_gain_turnOut45_700,&om_gain_turnOut45_700};

const static t_pid_gain sp_gain_turnIn135_700 = {16.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_700 = {0.8f, 0.00f, 0.0f};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_700_table = {0.70f, 35.0f,28.24+14.0,24.94+5.0, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_700_table = {0.70f,-35.0f,28.24+14.0,24.94+5.0,-135.0f,RIGHT};
const static t_param param_inL135_700 = {&slalom_inL135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700};
const static t_param param_inR135_700 = {&slalom_inR135_700_table,&sp_gain_turnIn135_700,&om_gain_turnIn135_700};

const static t_pid_gain sp_gain_turnOut135_700 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_700 = {0.8f, 0.00f, 0.0f};
const static t_turn_param_table slalom_outL135_700_table = {0.70f, 35.0f,20.56,32.69, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_700_table = {0.70f,-35.0f,20.56,32.69,-135.0f,RIGHT};
const static t_param param_outL135_700 = {&slalom_outL135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700};
const static t_param param_outR135_700 = {&slalom_outR135_700_table,&sp_gain_turnOut135_700,&om_gain_turnOut135_700};

const static t_param *const mode_700[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_700,		&param_L90_700,
												&param_R180_700,	&param_L180_700,
												&param_inR45_700,	&param_inL45_700,
												&param_outR45_700,	&param_outL45_700,
												&param_inR135_700,	&param_inL135_700,
												&param_outR135_700,	&param_outL135_700,
												&param_RV90_700,	&param_LV90_700
											};

//k = 100
//-----------velo = 700 mm/s parameters

const static t_pid_gain sp_gain_turn90_700_no_suction = {12.0,0.4,0.0};
const static t_pid_gain om_gain_turn90_700_no_suction = {0.4f, 0.00f, 0.00f};
const static t_turn_param_table slalom_L90_700_table_no_suction = {0.70f, 45.0f,21.90,35.04, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_700_table_no_suction = {0.70f,-45.0f,21.90,35.04,-90.0f,RIGHT};
const static t_param param_L90_700_no_suction = {&slalom_L90_700_table_no_suction ,&sp_gain_turn90_700_no_suction ,&om_gain_turn90_700_no_suction };
const static t_param param_R90_700_no_suction = {&slalom_R90_700_table_no_suction ,&sp_gain_turn90_700_no_suction ,&om_gain_turn90_700_no_suction };

const static t_pid_gain sp_gain_turn180_700_no_suction = {10.0,0.4,0.0};
const static t_pid_gain om_gain_turn180_700_no_suction = {0.4f, 0.0001f, 0.00f};//{1.0f, 0.0001f, 0.00f};
const static t_turn_param_table slalom_L180_700_table_no_suction = {0.70f, 45.0f,10.36,29.49, 180.0f,LEFT};
const static t_turn_param_table slalom_R180_700_table_no_suction = {0.70f,-45.0f,10.36,29.49,-180.0f,RIGHT};
const static t_param param_L180_700_no_suction = {&slalom_L180_700_table_no_suction,&sp_gain_turn180_700_no_suction,&om_gain_turn180_700_no_suction};
const static t_param param_R180_700_no_suction = {&slalom_R180_700_table_no_suction,&sp_gain_turn180_700_no_suction,&om_gain_turn180_700_no_suction};

const static t_pid_gain sp_gain_turnV90_700_no_suction = {10.0,0.1,0.0};
const static t_pid_gain om_gain_turnV90_700_no_suction = {0.6f, 0.0f, 0.0f};//0.8
//const static t_turn_param_table slalom_LV90_700_table = {0.70f, 35.0f,13.95,18.32, 90.0f,LEFT};
//const static t_turn_param_table slalom_RV90_700_table = {0.70f,-35.0f,13.95,18.32,-90.0f,RIGHT};
const static t_turn_param_table slalom_LV90_700_table_no_suction = {0.70f, 40.0f,8.95,10.32, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_700_table_no_suction = {0.70f,-40.0f,8.95,10.32,-90.0f,RIGHT};
const static t_param param_LV90_700_no_suction = {&slalom_LV90_700_table_no_suction,&sp_gain_turnV90_700_no_suction,&om_gain_turnV90_700_no_suction};
const static t_param param_RV90_700_no_suction = {&slalom_RV90_700_table_no_suction,&sp_gain_turnV90_700_no_suction,&om_gain_turnV90_700_no_suction};

const static t_pid_gain sp_gain_turnIn45_700_no_suction = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn45_700_no_suction = {0.6f, 0.001f, -0.0f};
const static t_turn_param_table slalom_inL45_700_table_no_suction = {0.70f, 52.0f,9.0,40.82, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_700_table_no_suction = {0.70f,-52.0f,9.0,40.82,-45.0f,RIGHT};
const static t_param param_inL45_700_no_suction = {&slalom_inL45_700_table_no_suction ,&sp_gain_turnIn45_700_no_suction ,&om_gain_turnIn45_700_no_suction };
const static t_param param_inR45_700_no_suction = {&slalom_inR45_700_table_no_suction ,&sp_gain_turnIn45_700_no_suction ,&om_gain_turnIn45_700_no_suction };

const static t_pid_gain sp_gain_turnOut45_700_no_suction = {12.0,0.3,0.0};
const static t_pid_gain om_gain_turnOut45_700_no_suction = {0.80f, 0.001f, 0.0f};
const static t_turn_param_table slalom_outL45_700_table_no_suction = {0.70f, 55.0f, 29.09,16.08, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_700_table_no_suction = {0.70f,-55.0f, 29.09,16.08,-45.0f,RIGHT};
const static t_param param_outL45_700_no_suction = {&slalom_outL45_700_table_no_suction,&sp_gain_turnOut45_700_no_suction,&om_gain_turnOut45_700_no_suction};
const static t_param param_outR45_700_no_suction = {&slalom_outR45_700_table_no_suction,&sp_gain_turnOut45_700_no_suction,&om_gain_turnOut45_700_no_suction};
//k = 70ぐらい
const static t_pid_gain sp_gain_turnIn135_700_no_suction = {16.0,0.4,0.0};
const static t_pid_gain om_gain_turnIn135_700_no_suction = {0.6f, 0.005f, 0.0f};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_700_table_no_suction = {0.70f, 40.0f,10,11, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_700_table_no_suction = {0.70f,-40.0f,10,11,-135.0f,RIGHT};
const static t_param param_inL135_700_no_suction = {&slalom_inL135_700_table_no_suction,&sp_gain_turnIn135_700_no_suction,&om_gain_turnIn135_700_no_suction};
const static t_param param_inR135_700_no_suction = {&slalom_inR135_700_table_no_suction,&sp_gain_turnIn135_700_no_suction,&om_gain_turnIn135_700_no_suction};

const static t_pid_gain sp_gain_turnOut135_700_no_suction = {14.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_700_no_suction = {0.6f, 0.01f, 0.0f};
const static t_turn_param_table slalom_outL135_700_table_no_suction = {0.70f, 40.0f,1.5,19.05, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_700_table_no_suction = {0.70f,-40.0f,1.5,19.05,-135.0f,RIGHT};
const static t_param param_outL135_700_no_suction = {&slalom_outL135_700_table_no_suction,&sp_gain_turnOut135_700_no_suction,&om_gain_turnOut135_700_no_suction};
const static t_param param_outR135_700_no_suction = {&slalom_outR135_700_table_no_suction,&sp_gain_turnOut135_700_no_suction,&om_gain_turnOut135_700_no_suction};

const static t_param *const mode_700_no_suction[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_700_no_suction ,			&param_L90_700_no_suction ,
												&param_R180_700_no_suction,			&param_L180_700_no_suction,
												&param_inR45_700_no_suction,		&param_inL45_700_no_suction,
												&param_outR45_700_no_suction,		&param_outL45_700_no_suction,
												&param_inR135_700_no_suction,		&param_inL135_700_no_suction,
												&param_outR135_700_no_suction,		&param_outL135_700_no_suction,
												&param_RV90_700_no_suction,			&param_LV90_700_no_suction
											};


//k = 150
//-----------velo = 1000 mm/s parameters

const static t_pid_gain sp_gain_turn90_1000 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn90_1000 = {0.75f, 0.0f, 0.00f};
const static t_turn_param_table slalom_L90_1000_table = {1.00f, 42.5f,24.92+5.0,38.40+10.0, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_1000_table = {1.00f,-42.5f,24.92+5.0,38.40+10.0,-90.0f,RIGHT};
const static t_param param_L90_1000 = {&slalom_L90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};
const static t_param param_R90_1000 = {&slalom_R90_1000_table,&sp_gain_turn90_1000,&om_gain_turn90_1000};

const static t_pid_gain sp_gain_turn180_1000 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turn180_1000 = {1.0f, 0.0f, 1.0f};
const static t_turn_param_table slalom_L180_1000_table = {1.00f, 47.5f,9.15+5.0,21.63+5.0, 175.0f,LEFT};
const static t_turn_param_table slalom_R180_1000_table = {1.00f,-47.5f,9.15+5.0,21.63+5.0,-175.0f,RIGHT};
const static t_param param_L180_1000 = {&slalom_L180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};
const static t_param param_R180_1000 = {&slalom_R180_1000_table,&sp_gain_turn180_1000,&om_gain_turn180_1000};

//not adjust
const static t_pid_gain sp_gain_turnV90_1000 = {16.0,0.3,0.0};
const static t_pid_gain om_gain_turnV90_1000 = {0.8f, 0.0f, 1.5f};
const static t_turn_param_table slalom_LV90_1000_table = {1.00f, 37.5f,9.64+3.0,15.51+4.0, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_1000_table = {1.00f,-37.5f,9.64+3.0,15.51+4.0,-90.0f,RIGHT};
const static t_param param_LV90_1000 = {&slalom_LV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};
const static t_param param_RV90_1000 = {&slalom_RV90_1000_table,&sp_gain_turnV90_1000,&om_gain_turnV90_1000};

const static t_pid_gain sp_gain_turnIn45_1000 = {16.0,0.3,0.0};
const static t_pid_gain om_gain_turnIn45_1000 = {0.8f, 0.00f, 1.0f};
const static t_turn_param_table slalom_inL45_1000_table = {1.00f, 50.0f,12.56,36.53, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_1000_table = {1.00f,-50.0f,12.56,36.53,-45.0f,RIGHT};
const static t_param param_inL45_1000 = {&slalom_inL45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};
const static t_param param_inR45_1000 = {&slalom_inR45_1000_table,&sp_gain_turnIn45_1000,&om_gain_turnIn45_1000};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1000 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut45_1000 = {0.6f, 0.0f, 0.5f};
const static t_turn_param_table slalom_outL45_1000_table = {1.00f, 45.0f,34.04,21.50, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_1000_table = {1.00f,-45.0f,34.04,21.50,-45.0f,RIGHT};
const static t_param param_outL45_1000 = {&slalom_outL45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};
const static t_param param_outR45_1000 = {&slalom_outR45_1000_table,&sp_gain_turnOut45_1000,&om_gain_turnOut45_1000};


const static t_pid_gain sp_gain_turnIn135_1000 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnIn135_1000 = {0.8f, 0.0f, 0.0f};//{0.7f, 0.7f, 0.0f};
const static t_turn_param_table slalom_inL135_1000_table = {1.00f, 37.5f,15.31+13.0,20.21+9.0, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_1000_table = {1.00f,-37.5f,15.31+13.0,20.21+9.0,-135.0f,RIGHT};
const static t_param param_inL135_1000 = {&slalom_inL135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};
const static t_param param_inR135_1000 = {&slalom_inR135_1000_table,&sp_gain_turnIn135_1000,&om_gain_turnIn135_1000};

//
const static t_pid_gain sp_gain_turnOut135_1000 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_1000 = {0.8f, 0.00f, 0.0f};
const static t_turn_param_table slalom_outL135_1000_table = {1.00f, 37.5f,12.15+15.0,25.76+25.0, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_1000_table = {1.00f,-37.5f,12.15+15.0,25.76+25.0,-135.0f,RIGHT};
const static t_param param_outL135_1000 = {&slalom_outL135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};
const static t_param param_outR135_1000 = {&slalom_outR135_1000_table,&sp_gain_turnOut135_1000,&om_gain_turnOut135_1000};

const static t_param *const mode_1000[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1000,		&param_L90_1000,
												&param_R180_1000,	&param_L180_1000,
												&param_inR45_1000,	&param_inL45_1000,
												&param_outR45_1000,	&param_outL45_1000,
												&param_inR135_1000,	&param_inL135_1000,
												&param_outR135_1000,	&param_outL135_1000,
												&param_RV90_1000,	&param_LV90_1000
											};

//k = 300
//-----------velo = 1200 mm/s parameters

const static t_pid_gain sp_gain_turn90_1200 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turn90_1200 = {0.75f, 0.01f, 0.00f};
const static t_turn_param_table slalom_L90_1200_table = {1.20f, 50.0f,20.11,30.83, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_1200_table = {1.20f,-50.0f,20.11,30.83,-90.0f,RIGHT};
const static t_param param_L90_1200 = {&slalom_L90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200};
const static t_param param_R90_1200 = {&slalom_R90_1200_table,&sp_gain_turn90_1200,&om_gain_turn90_1200};

const static t_pid_gain sp_gain_turn180_1200 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turn180_1200 = {0.9f, 0.02f, 0.00f};//{0.60f, 0.02f, 0.001f};
//const static t_turn_param_table slalom_L180_1200_table = {1.20f, 42.0f,18.70,23.5+0.0, 180.0f,LEFT};
//const static t_turn_param_table slalom_R180_1200_table = {1.20f,-47.5f,19.98,28.15+5.0,-175.0f,RIGHT};

const static t_turn_param_table slalom_L180_1200_table = {1.20f, 45.0f,18.13,28.55, 180.0f,LEFT};
const static t_turn_param_table slalom_R180_1200_table = {1.20f,-45.0f,18.13,28.55,-180.0+0.0f,RIGHT};

const static t_param param_L180_1200 = {&slalom_L180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200};
const static t_param param_R180_1200 = {&slalom_R180_1200_table,&sp_gain_turn180_1200,&om_gain_turn180_1200};

//not adjust
const static t_pid_gain sp_gain_turnV90_1200 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turnV90_1200 = {1.0f, 0.01f, 1.0f};//{1.0f, 0.01f, 0.0f};//r,2.20.0005f,0.3
const static t_turn_param_table slalom_LV90_1200_table = {1.20f, 40.0f,7.57,17.53, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_1200_table = {1.20f,-40.0f,7.57,17.53,-90.0f,RIGHT};
const static t_param param_LV90_1200 = {&slalom_LV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};
const static t_param param_RV90_1200 = {&slalom_RV90_1200_table,&sp_gain_turnV90_1200,&om_gain_turnV90_1200};

//adjust -> 11/06
const static t_pid_gain sp_gain_turnIn45_1200 = {14.0,0.6,0.0};
//const static t_pid_gain om_gain_turnIn45_1200 = {0.75f, 0.001f, 2.8};//{0.45f, 0.005f, 0.04f};//{1.5f, 0.001f, 0.008f}};
const static t_pid_gain om_gain_turnIn45_1200 = {0.8f, 0.01f, 0.0f};//r 55->48
//const static t_turn_param_table slalom_inL45_1200_table = {1.20f, 62.5f,5.13,30.33, 45.0f,LEFT};
//const static t_turn_param_table slalom_inR45_1200_table = {1.20f,-62.5f,5.13,30.33,-45.0f,RIGHT};
const static t_turn_param_table slalom_inL45_1200_table = {1.20f, 55.0f,9.15,37.06, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_1200_table = {1.20f,-55.0f,9.15,37.06,-45.0f,RIGHT};
const static t_param param_inL45_1200 = {&slalom_inL45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};
const static t_param param_inR45_1200 = {&slalom_inR45_1200_table,&sp_gain_turnIn45_1200,&om_gain_turnIn45_1200};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1200 = {14.0,0.6,0.0};
//const static t_pid_gain om_gain_turnOut45_1200 = {1.5f, 0.001f, 1.0f};
const static t_pid_gain om_gain_turnOut45_1200 = {0.8f, 0.001f, 0.1f};
//const static t_pid_gain om_gain_turnOut45_1200 = {0.75f, 0.001f, 2.8};
const static t_turn_param_table slalom_outL45_1200_table = {1.20f, 55.0f,27.84,18.40, 45.0f,LEFT};//{1.20f, 55.0f,39.17,16.26, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_1200_table = {1.20f,-55.0f,27.84,18.40,-45.0f,RIGHT};//{1.20f,-55.0f,39.17,16.26,-45.0f,RIGHT};
const static t_param param_outL45_1200 = {&slalom_outL45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200};
const static t_param param_outR45_1200 = {&slalom_outR45_1200_table,&sp_gain_turnOut45_1200,&om_gain_turnOut45_1200};


const static t_pid_gain sp_gain_turnIn135_1200 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turnIn135_1200 = {0.7f, 0.005f, 0.1f};//{0.70f, 0.00f, 1.5f};

const static t_turn_param_table slalom_inL135_1200_table = {1.20f, 40.0f,17.02,19.82, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_1200_table = {1.20f,-40.0f,17.02,19.82,-135.0f,RIGHT};

const static t_param param_inL135_1200 = {&slalom_inL135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200};
const static t_param param_inR135_1200 = {&slalom_inR135_1200_table,&sp_gain_turnIn135_1200,&om_gain_turnIn135_1200};

//start: 11.26148848963036 ,end: 26.22220872831427
const static t_pid_gain sp_gain_turnOut135_1200 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_1200 = {0.8f, 0.00f, 0.0f};
const static t_turn_param_table slalom_outL135_1200_table = {1.20f, 40.0f,9.37,27.58, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_1200_table = {1.20f,-40.0f,9.37,27.58,-135.0f,RIGHT};
const static t_param param_outL135_1200 = {&slalom_outL135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200};
const static t_param param_outR135_1200 = {&slalom_outR135_1200_table,&sp_gain_turnOut135_1200,&om_gain_turnOut135_1200};

const static t_param *const mode_1200[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1200,		&param_L90_1200,
												&param_R180_1200,	&param_L180_1200,
												&param_inR45_1200,	&param_inL45_1200,
												&param_outR45_1200,	&param_outL45_1200,
												&param_inR135_1200,	&param_inL135_1200,
												&param_outR135_1200,	&param_outL135_1200,
												&param_RV90_1200,	&param_LV90_1200
											};

//
//-----------velo = 1300 mm/s parameters

const static t_pid_gain sp_gain_turn90_1300 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turn90_1300 = {0.75f, 0.01f, 0.00f};
const static t_turn_param_table slalom_L90_1300_table = {1.30f, 55.0f,12.46+10.0,19.17, 90.0f,LEFT};
const static t_turn_param_table slalom_R90_1300_table = {1.30f,-55.0f,12.46+10.0,19.17,-90.0f,RIGHT};
const static t_param param_L90_1300 = {&slalom_L90_1300_table,&sp_gain_turn90_1300,&om_gain_turn90_1300};
const static t_param param_R90_1300 = {&slalom_R90_1300_table,&sp_gain_turn90_1300,&om_gain_turn90_1300};

const static t_pid_gain sp_gain_turn180_1300 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turn180_1300 = {0.6f, 0.02f, 0.00f};//{0.80f, 0.02f, 0.00f};
//const static t_turn_param_table slalom_L180_1200_table = {1.20f, 42.0f,18.70,23.5+0.0, 180.0f,LEFT};
//const static t_turn_param_table slalom_R180_1200_table = {1.20f,-47.5f,19.98,28.15+5.0,-175.0f,RIGHT};

const static t_turn_param_table slalom_L180_1300_table = {1.30f, 43.0f,20.37,25.91+0.0, 180.0f,LEFT};
const static t_turn_param_table slalom_R180_1300_table = {1.30f,-43.0f,20.37,25.91+0.0,-180.0+0.0f,RIGHT};

const static t_param param_L180_1300 = {&slalom_L180_1300_table,&sp_gain_turn180_1300,&om_gain_turn180_1300};
const static t_param param_R180_1300 = {&slalom_R180_1300_table,&sp_gain_turn180_1300,&om_gain_turn180_1300};

//not adjust
const static t_pid_gain sp_gain_turnV90_1300 = {14.0,0.6,0.0};
const static t_pid_gain om_gain_turnV90_1300 = {1.00f, 0.05f, 2.0f};//1.0f, 0.005f, 0.0f{1.0f, 0.01f, 0.0f};//r,2.20.0005f,0.3
const static t_turn_param_table slalom_LV90_1300_table = {1.30f, 43.0f,6.27,11.82, 90.0f,LEFT};
const static t_turn_param_table slalom_RV90_1300_table = {1.30f,-43.0f,6.27,11.82,-90.0f,RIGHT};
const static t_param param_LV90_1300 = {&slalom_LV90_1300_table,&sp_gain_turnV90_1300,&om_gain_turnV90_1300};
const static t_param param_RV90_1300 = {&slalom_RV90_1300_table,&sp_gain_turnV90_1300,&om_gain_turnV90_1300};

//adjust -> 11/06
const static t_pid_gain sp_gain_turnIn45_1300 = {14.0,0.6,0.0};
//const static t_pid_gain om_gain_turnIn45_1300 = {0.75f, 0.001f, 2.8};//{0.45f, 0.005f, 0.04f};//{1.5f, 0.001f, 0.008f}};
const static t_pid_gain om_gain_turnIn45_1300 = {0.5f, 0.0f, -0.2f};//r 55->48
//const static t_turn_param_table slalom_inL45_1300_table = {1.30f, 62.5f,5.13,30.33, 45.0f,LEFT};
//const static t_turn_param_table slalom_inR45_1300_table = {1.30f,-62.5f,5.13,30.33,-45.0f,RIGHT};
const static t_turn_param_table slalom_inL45_1300_table = {1.30f, 45.0f,6.9,35.31, 45.0f,LEFT};
const static t_turn_param_table slalom_inR45_1300_table = {1.30f,-45.0f,6.9,35.31,-45.0f,RIGHT};
const static t_param param_inL45_1300 = {&slalom_inL45_1300_table,&sp_gain_turnIn45_1300,&om_gain_turnIn45_1300};
const static t_param param_inR45_1300 = {&slalom_inR45_1300_table,&sp_gain_turnIn45_1300,&om_gain_turnIn45_1300};

//k = 300
const static t_pid_gain sp_gain_turnOut45_1300 = {14.0,0.6,0.0};
//const static t_pid_gain om_gain_turnOut45_1300 = {1.5f, 0.001f, 1.0f};
const static t_pid_gain om_gain_turnOut45_1300 = {0.8f, 0.001f, 0.1f};
//const static t_pid_gain om_gain_turnOut45_1300 = {0.75f, 0.001f, 2.8};
const static t_turn_param_table slalom_outL45_1300_table = {1.30f, 50.0f,31.075,18.064, 45.0f,LEFT};//{1.30f, 55.0f,39.17,16.26, 45.0f,LEFT};
const static t_turn_param_table slalom_outR45_1300_table = {1.30f,-50.0f,31.075,18.064,-45.0f,RIGHT};//{1.30f,-55.0f,39.17,16.26,-45.0f,RIGHT};
const static t_param param_outL45_1300 = {&slalom_outL45_1300_table,&sp_gain_turnOut45_1300,&om_gain_turnOut45_1300};
const static t_param param_outR45_1300 = {&slalom_outR45_1300_table,&sp_gain_turnOut45_1300,&om_gain_turnOut45_1300};


const static t_pid_gain sp_gain_turnIn135_1300 = {14.0,0.6,0.1};
const static t_pid_gain om_gain_turnIn135_1300 = {0.8f, 0.02f, 0.0f};//0.7f, 0.005f, 0.1f{0.70f, 0.00f, 1.5f};

const static t_turn_param_table slalom_inL135_1300_table = {1.30f, 40.0f,4.91,14.77, 135.0f,LEFT};
const static t_turn_param_table slalom_inR135_1300_table = {1.30f,-40.0f,4.91,14.77,-135.0f,RIGHT};

const static t_param param_inL135_1300 = {&slalom_inL135_1300_table,&sp_gain_turnIn135_1300,&om_gain_turnIn135_1300};
const static t_param param_inR135_1300 = {&slalom_inR135_1300_table,&sp_gain_turnIn135_1300,&om_gain_turnIn135_1300};

//start: 11.26148848963036 ,end: 26.22220872831427
const static t_pid_gain sp_gain_turnOut135_1300 = {12.0,0.1,0.0};
const static t_pid_gain om_gain_turnOut135_1300 = {0.8f, 0.00f, 0.0f};
const static t_turn_param_table slalom_outL135_1300_table = {1.30f, 40.0f,4.70+7.0,18.62+5.0, 135.0f,LEFT};
const static t_turn_param_table slalom_outR135_1300_table = {1.30f,-40.0f,4.70+7.0,18.62+5.0,-135.0f,RIGHT};
const static t_param param_outL135_1300 = {&slalom_outL135_1300_table,&sp_gain_turnOut135_1300,&om_gain_turnOut135_1300};
const static t_param param_outR135_1300 = {&slalom_outR135_1300_table,&sp_gain_turnOut135_1300,&om_gain_turnOut135_1300};

const static t_param *const mode_1300[] = 	{	&param_dummy,		&param_dummy,		&param_dummy,
												&param_R90_1300,		&param_L90_1300,
												&param_R180_1300,	&param_L180_1300,
												&param_inR45_1300,	&param_inL45_1300,
												&param_outR45_1300,	&param_outL45_1300,
												&param_inR135_1300,	&param_inL135_1300,
												&param_outR135_1300,	&param_outL135_1300,
												&param_RV90_1300,	&param_LV90_1300
											};



const static t_turn_param_table slalom_L90_table_v3 = {1.0f,48.0f,25.0,25.0,90.0f,LEFT};

//const static t_pid_gain sp_gain_300 = {12.0,0.2,0.0};
//const static t_pid_gain om_gain_300 = {0.0,0.0,0.0};
//const static t_param param_300 = {&slalom_outL135_300_table,&sp_gain_300,&om_gain_300};
//const t_turn_param_table L90 = {0.30f,25.0f,11.0,11.0,90.0f,LEFT};
//const t_pid_gain		 L90_velo_gain = {1.0,0.2,0.0f};
//const t_pid_gain		 L90_rad_gain = {1.0,0.2,0.0f};
//static t_param p ={L90,L90_velo_gain,L90_rad_gain};

#endif /* MODULE_INC_RUN_PRAM_C_ */
