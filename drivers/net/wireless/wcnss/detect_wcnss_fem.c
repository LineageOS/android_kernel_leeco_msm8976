/**
*File name: detect_wcnss_fem.c
*Author: liuyongkang
*Date: 20160321
*Purpose: To check whether WiFi with FEM
*/

#include <linux/syscalls.h>
#include "detect_wcnss_fem.h"

#ifdef COMPATIBLE_WCNSS_NV

static volatile int wcnss_wlan_fem;
static char board_id_buffer[BOARD_ID_SIZE+1];
static const char board_id_table_wo_fem[BOARD_ID_NUM_WO_FEM][BOARD_ID_SIZE+1] = {
    {"V0.0.0"},
    {"V0.0.1"},
    {"V1.0.0"},
    {"V2.0.0"},
    {"V2.0.1"},
    {"V2.0.2"},
};

void detect_wcnss_fem(void)
{
    int i;
    int val;

    obtain_hw_board_id((char*)board_id_buffer);

    for(i=0; i<BOARD_ID_NUM_WO_FEM; i++)
    {
        val = strncmp((char*)board_id_buffer, (char*)&board_id_table_wo_fem[i], BOARD_ID_SIZE);
        if (!val)
        {
            pr_info("This wcnss without fem !!!");
            wcnss_wlan_fem = WCNSS_WITHOUT_FEM;
            return;
        }
    }

    pr_info("This wcnss with fem !!!");
    wcnss_wlan_fem = WCNSS_WITH_FEM;
    return;
}

int if_wcnss_with_fem(void)
{
    return wcnss_wlan_fem;
}

EXPORT_SYMBOL(if_wcnss_with_fem);

#endif
