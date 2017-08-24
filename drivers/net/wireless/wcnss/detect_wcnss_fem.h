/**
*File name: detect_wcnss_fem.h
*Author: liuyongkang
*Date: 20160321
*Purpose: To check whether WiFi with FEM
*/

#define COMPATIBLE_WCNSS_NV

#ifdef COMPATIBLE_WCNSS_NV
#define BOARD_ID_SIZE 6
#define BOARD_ID_NUM_WO_FEM 6

enum wcnss_fem{
	WCNSS_WITHOUT_FEM,
	WCNSS_WITH_FEM,
	WCNSS_INVALID_FEM,
};

extern void detect_wcnss_fem(void);
extern int if_wcnss_with_fem(void);
extern void obtain_hw_board_id(char *buf);
#endif
