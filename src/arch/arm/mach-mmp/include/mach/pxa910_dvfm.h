/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef PXA910_DVFM_H
#define PXA910_DVFM_H

#include <mach/dvfm.h>

#define OP_NAME_LEN		16

struct pxa910_md_opt {
	int	vcc_core;	/* core voltage */
	int	pclk;		/* core clock */
	int	pdclk;		/* DDR interface clock */
	int	baclk;		/* bus interface clock */
	int	xpclk;		/* L2 cache interface clock */
	int	dclk;		/* DDR clock */
	int	aclk;		/* bus clock */
	int	lpj;
	char	name[OP_NAME_LEN];
};

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
#define BIT_4 (1 << 4)
#define BIT_5 (1 << 5)
#define BIT_6 (1 << 6)
#define BIT_7 (1 << 7)
#define BIT_8 (1 << 8)
#define BIT_9 (1 << 9)
#define BIT_10 (1 << 10)
#define BIT_11 (1 << 11)
#define BIT_12 (1 << 12)
#define BIT_13 (1 << 13)
#define BIT_14 (1 << 14)
#define BIT_15 (1 << 15)
#define BIT_16 (1 << 16)
#define BIT_17 (1 << 17)
#define BIT_18 (1 << 18)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_21 (1 << 21)
#define BIT_22 (1 << 22)
#define BIT_23 (1 << 23)
#define BIT_24 (1 << 24)
#define BIT_25 (1 << 25)
#define BIT_26 (1 << 26)
#define BIT_27 (1 << 27)
#define BIT_28 (1 << 28)
#define BIT_29 (1 << 29)
#define BIT_30 (1 << 30)
#define BIT_31 ((unsigned)1 << 31)

#define SHIFT0(Val)  (Val)
#define SHIFT1(Val)  ((Val) << 1)
#define SHIFT2(Val)  ((Val) << 2)
#define SHIFT3(Val)  ((Val) << 3)
#define SHIFT4(Val)  ((Val) << 4)
#define SHIFT5(Val)  ((Val) << 5)
#define SHIFT6(Val)  ((Val) << 6)
#define SHIFT7(Val)  ((Val) << 7)
#define SHIFT8(Val)  ((Val) << 8)
#define SHIFT9(Val)  ((Val) << 9)
#define SHIFT10(Val) ((Val) << 10)
#define SHIFT11(Val) ((Val) << 11)
#define SHIFT12(Val) ((Val) << 12)
#define SHIFT13(Val) ((Val) << 13)
#define SHIFT14(Val) ((Val) << 14)
#define SHIFT15(Val) ((Val) << 15)
#define SHIFT16(Val) ((Val) << 16)
#define SHIFT17(Val) ((Val) << 17)
#define SHIFT18(Val) ((Val) << 18)
#define SHIFT19(Val) ((Val) << 19)
#define SHIFT20(Val) ((Val) << 20)
#define SHIFT21(Val) ((Val) << 21)
#define SHIFT22(Val) ((Val) << 22)
#define SHIFT23(Val) ((Val) << 23)
#define SHIFT24(Val) ((Val) << 24)
#define SHIFT25(Val) ((Val) << 25)
#define SHIFT26(Val) ((Val) << 26)
#define SHIFT27(Val) ((Val) << 27)
#define SHIFT28(Val) ((Val) << 28)
#define SHIFT29(Val) ((Val) << 29)
#define SHIFT30(Val) ((Val) << 30)
#define SHIFT31(Val) ((Val) << 31)

/*
 * pmua registers and bits definition
 */
#define	CC_SEA_OFF				0x0000
#define	CC_MOH_OFF				0x0004
#define	DM_CC_SEA_OFF				0x0008
#define	DM_CC_MOH_OFF				0x000C
#define	MOH_IMR_OFF				0x0098
#define	MOH_ISR_OFF				0x00A0

#define	PMUA_CC_SEA_SEA_RD_ST_CLEAR		BIT_31
#define	PMUA_CC_SEA_ACLK_DYN_FC			BIT_30
#define	PMUA_CC_SEA_DCLK_DYN_FC			BIT_29
#define	PMUA_CC_SEA_CORE_DYN_FC			BIT_28
#define	PMUA_CC_SEA_SEA_ALLOW_SPD_CHG		BIT_27
#define	PMUA_CC_SEA_BUS_FREQ_CHG_REQ		BIT_26
#define	PMUA_CC_SEA_DDR_FREQ_CHG_REQ		BIT_25
#define	PMUA_CC_SEA_SEA_FREQ_CHG_REQ		BIT_24
#define	PMUA_CC_SEA_ASYNC5			BIT_23
#define	PMUA_CC_SEA_ASYNC4			BIT_22
#define	PMUA_CC_SEA_ASYNC3_1			BIT_21
#define	PMUA_CC_SEA_ASYNC3			BIT_20
#define	PMUA_CC_SEA_ASYNC2			BIT_19
#define	PMUA_CC_SEA_ASYNC1			BIT_18
#define	PMUA_CC_SEA_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_CC_SEA_BUS_CLK_DIV_BASE		15
#define	PMUA_CC_SEA_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_CC_SEA_DDR_CLK_DIV_BASE		12
#define	PMUA_CC_SEA_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_CC_SEA_XP_CLK_DIV_BASE		9
#define	PMUA_CC_SEA_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_CC_SEA_BIU_CLK_DIV_BASE		6
#define	PMUA_CC_SEA_BUS_MC_CLK_DIV_MSK		SHIFT3(0x7)
#define	PMUA_CC_SEA_BUS_MC_CLK_DIV_BASE		3
#define	PMUA_CC_SEA_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_CC_SEA_CORE_CLK_DIV_BASE		0

#define	PMUA_CC_MOH_MOH_RD_ST_CLEAR		BIT_31
#define	PMUA_CC_MOH_ACLK_DYN_FC			BIT_30
#define	PMUA_CC_MOH_DCLK_DYN_FC			BIT_29
#define	PMUA_CC_MOH_CORE_DYN_FC			BIT_28
#define	PMUA_CC_MOH_MOH_ALLOW_SPD_CHG		BIT_27
#define	PMUA_CC_MOH_BUS_FREQ_CHG_REQ		BIT_26
#define	PMUA_CC_MOH_DDR_FREQ_CHG_REQ		BIT_25
#define	PMUA_CC_MOH_MOH_FREQ_CHG_REQ		BIT_24
#define	PMUA_CC_MOH_ASYNC5			BIT_23
#define	PMUA_CC_MOH_ASYNC4			BIT_22
#define	PMUA_CC_MOH_ASYNC3_1			BIT_21
#define	PMUA_CC_MOH_ASYNC3			BIT_20
#define	PMUA_CC_MOH_ASYNC2			BIT_19
#define	PMUA_CC_MOH_ASYNC1			BIT_18
#define	PMUA_CC_MOH_BUS_2_CLK_DIV_BASE		18
#define	PMUA_CC_MOH_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_CC_MOH_BUS_CLK_DIV_BASE		15
#define	PMUA_CC_MOH_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_CC_MOH_DDR_CLK_DIV_BASE		12
#define	PMUA_CC_MOH_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_CC_MOH_XP_CLK_DIV_BASE		9
#define	PMUA_CC_MOH_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_CC_MOH_BIU_CLK_DIV_BASE		6
#define	PMUA_CC_MOH_BUS_MC_CLK_DIV_MSK		SHIFT3(0x7)
#define	PMUA_CC_MOH_BUS_MC_CLK_DIV_BASE		3
#define	PMUA_CC_MOH_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_CC_MOH_CORE_CLK_DIV_BASE		0

#define	PMUA_DM_CC_SEA_MOH_RD_STATUS		BIT_25
#define	PMUA_DM_CC_SEA_SEA_RD_STATUS		BIT_24
#define	PMUA_DM_CC_SEA_ASYNC5			BIT_23
#define	PMUA_DM_CC_SEA_ASYNC4			BIT_22
#define	PMUA_DM_CC_SEA_ASYNC3_1			BIT_21
#define	PMUA_DM_CC_SEA_ASYNC3			BIT_20
#define	PMUA_DM_CC_SEA_ASYNC2			BIT_19
#define	PMUA_DM_CC_SEA_ASYNC1			BIT_18
#define	PMUA_DM_CC_SEA_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_DM_CC_SEA_BUS_CLK_DIV_BASE		15
#define	PMUA_DM_CC_SEA_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_DM_CC_SEA_DDR_CLK_DIV_BASE		12
#define	PMUA_DM_CC_SEA_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_DM_CC_SEA_XP_CLK_DIV_BASE		9
#define	PMUA_DM_CC_SEA_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_DM_CC_SEA_BIU_CLK_DIV_BASE		6
#define	PMUA_DM_CC_SEA_BUS_MC_CLK_DIV_MSK	SHIFT3(0x7)
#define	PMUA_DM_CC_SEA_BUS_MC_CLK_DIV_BASE	3
#define	PMUA_DM_CC_SEA_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_DM_CC_SEA_CORE_CLK_DIV_BASE	0

#define	PMUA_DM_CC_MOH_MOH_RD_STATUS		BIT_25
#define	PMUA_DM_CC_MOH_SEA_RD_STATUS		BIT_24
#define	PMUA_DM_CC_MOH_ASYNC5			BIT_23
#define	PMUA_DM_CC_MOH_ASYNC4			BIT_22
#define	PMUA_DM_CC_MOH_ASYNC3_1			BIT_21
#define	PMUA_DM_CC_MOH_ASYNC3			BIT_20
#define	PMUA_DM_CC_MOH_ASYNC2			BIT_19
#define	PMUA_DM_CC_MOH_ASYNC1			BIT_18
#define	PMUA_DM_CC_MOH_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	PMUA_DM_CC_MOH_BUS_CLK_DIV_BASE		15
#define	PMUA_DM_CC_MOH_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	PMUA_DM_CC_MOH_DDR_CLK_DIV_BASE		12
#define	PMUA_DM_CC_MOH_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	PMUA_DM_CC_MOH_XP_CLK_DIV_BASE		9
#define	PMUA_DM_CC_MOH_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	PMUA_DM_CC_MOH_BIU_CLK_DIV_BASE		6
#define	PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_MSK	SHIFT3(0x7)
#define	PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_BASE	3
#define	PMUA_DM_CC_MOH_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	PMUA_DM_CC_MOH_CORE_CLK_DIV_BASE	0

#define	PMUA_MOH_IMR_MOH_FC_INTR_MASK		BIT_1
#define	PMUA_MOH_IMR_SEA_FC_INTR_MASK		BIT_0

#define	PMUA_MOH_ISR_MOH_FC_ISR			BIT_1
#define	PMUA_MOH_ISR_SEA_FC_ISR			BIT_0

/*
 * pmum registers and bits definition
 */
#define	FCCR_OFF				0x0008

#define	PMUM_FCCR_MOHCLKSEL_MSK			SHIFT29(0x7)
#define	PMUM_FCCR_MOHCLKSEL_BASE		29
#define	PMUM_FCCR_SEAGCLKSEL_MSK		SHIFT26(0x7)
#define	PMUM_FCCR_SEAGCLKSEL_BASE		26
#define	PMUM_FCCR_AXICLKSEL_MSK			SHIFT23(0x7)
#define	PMUM_FCCR_AXICLKSEL_BASE		23
#define	PMUM_FCCR_MFC				BIT_15
#define	PMUM_FCCR_PLL1CEN			BIT_14
#define	PMUM_FCCR_PLL1REFD_MSK			SHIFT9(0x1f)
#define	PMUM_FCCR_PLL1REFD_BASE			9
#define	PMUM_FCCR_PLL1FBD_MSK			SHIFT0(0x1ff)
#define	PMUM_FCCR_PLL1FBD_BASE			0

#endif
